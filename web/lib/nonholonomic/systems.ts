/**
 * The chapter's control systems as vector fields with analytic Jacobians:
 * Choset's unicycle (Examples 12.1.1–12.1.6), Hitch's car (eq. 12.34, state
 * ordering q = (x, y, φ, θ) — steering angle *before* heading, as Choset
 * writes it), the car with one trailer (eq. 12.38, written with the trailer's
 * absolute heading ψ, which is Chapter 2's `HitchCfg.trailer`), the planar
 * body with thrusters (Examples 12.1.2–12.1.8), the foliation of Example
 * 12.1.7, and the small counterexamples §12.3 lists.
 *
 * The hand-derived brackets live beside the fields they belong to, so the
 * checks can hold `lieBracket` (finite differences) against the formulas Choset
 * prints — the micro-example of the design doc is exactly that comparison.
 */

import type { Mat, Vec } from '../prob/linalg';
import { coordinateField, field, type VectorField } from './field';

const zeros = (n: number): Mat => Array.from({ length: n }, () => new Array<number>(n).fill(0));

/* -------------------------------------------------------------------------- */
/* Unicycle (Choset Example 12.1.1): x = (x₁, x₂, x₃), drive and spin          */
/* -------------------------------------------------------------------------- */

export interface Unicycle {
  /** g₁ = [cos x₃, sin x₃, 0]ᵀ — rolling forward at unit speed. */
  drive: VectorField;
  /** g₂ = [0, 0, 1]ᵀ — spinning counterclockwise at unit speed. */
  spin: VectorField;
  /** Hand formula: [g₁, g₂] = [sin x₃, −cos x₃, 0]ᵀ (Example 12.1.5) — sideways, the forbidden direction. */
  bracketExact: (x: Vec) => Vec;
  /** The exact four-flow residue ε[cos x₃ − cos(x₃ + ε), sin x₃ − sin(x₃ + ε), 0] (Derivation 1). */
  loopExact: (x: Vec, eps: number) => Vec;
  /** ω(x) = [−sin x₃, cos x₃, 0]: the annihilating covector of (12.1). */
  constraintCovector: (x: Vec) => Vec;
}

export function unicycle(): Unicycle {
  const drive = field(
    'g1',
    3,
    (x) => [Math.cos(x[2]), Math.sin(x[2]), 0],
    (x) => [
      [0, 0, -Math.sin(x[2])],
      [0, 0, Math.cos(x[2])],
      [0, 0, 0],
    ],
  );
  const spin = coordinateField(3, 2, 'g2');
  return {
    drive,
    spin,
    bracketExact: (x) => [Math.sin(x[2]), -Math.cos(x[2]), 0],
    loopExact: (x, eps) => [eps * (Math.cos(x[2]) - Math.cos(x[2] + eps)), eps * (Math.sin(x[2]) - Math.sin(x[2] + eps)), 0],
    constraintCovector: (x) => [-Math.sin(x[2]), Math.cos(x[2]), 0],
  };
}

/* -------------------------------------------------------------------------- */
/* The car (Choset eq. 12.34): q = (x, y, φ, θ), inputs speed and steering rate  */
/* -------------------------------------------------------------------------- */

export interface CarFields {
  L: number;
  /** g₁ = [cos θ, sin θ, 0, tan φ / L]ᵀ. */
  drive: VectorField;
  /** g₂ = ∂/∂φ. */
  steer: VectorField;
  /** g₃ = [g₁, g₂] = [0, 0, 0, −sec²φ/L]ᵀ: the heading direction, bought by one bracket. */
  g3Exact: (q: Vec) => Vec;
  /** g₄ = [g₁, g₃] = (sec²φ/L)[−sin θ, cos θ, 0, 0]ᵀ: sideways, bought by two. */
  g4Exact: (q: Vec) => Vec;
  /** det[g₁ g₂ g₃ g₄] = −sec⁴φ/L², nonzero for |φ| < π/2. */
  detExact: (q: Vec) => number;
}

export function car(L = 1): CarFields {
  const drive = field(
    'g1',
    4,
    (q) => [Math.cos(q[3]), Math.sin(q[3]), 0, Math.tan(q[2]) / L],
    (q) => {
      const J = zeros(4);
      J[0][3] = -Math.sin(q[3]);
      J[1][3] = Math.cos(q[3]);
      const sec2 = 1 / Math.cos(q[2]) ** 2;
      J[3][2] = sec2 / L;
      return J;
    },
  );
  const steer = coordinateField(4, 2, 'g2');
  const sec2 = (q: Vec) => 1 / Math.cos(q[2]) ** 2;
  return {
    L,
    drive,
    steer,
    g3Exact: (q) => [0, 0, 0, -sec2(q) / L],
    g4Exact: (q) => {
      const s = sec2(q) / L;
      return [-s * Math.sin(q[3]), s * Math.cos(q[3]), 0, 0];
    },
    detExact: (q) => -(sec2(q) ** 2) / (L * L),
  };
}

/**
 * The reduced car (Choset eq. 12.35): the steering angle moves into the control
 * and the state is the body pose (x, y, θ). Identical to the unicycle as fields;
 * only the control set differs (|ω| ≤ |v|/ρ). Chapter 21's Dubins and
 * Reeds–Shepp live on this system.
 */
export function reducedCar(): { drive: VectorField; turn: VectorField } {
  const u = unicycle();
  return { drive: u.drive, turn: u.spin };
}

/* -------------------------------------------------------------------------- */
/* Car with one trailer: q = (x, y, φ, θ, ψ), ψ the trailer's absolute heading  */
/* -------------------------------------------------------------------------- */

export interface CarTrailerFields {
  L: number;
  d: number;
  /** g₁ = [cos θ, sin θ, 0, tan φ/L, sin(θ − ψ)/d]ᵀ — Chapter 2's `Hitch.step` physics. */
  drive: VectorField;
  steer: VectorField;
  /** The hitch angle θ₁ = θ − ψ, Choset's relative trailer coordinate. */
  hitchAngle: (q: Vec) => number;
}

/**
 * Choset's eq. (12.38) uses the relative angle θ₁; Chapter 2's simulator
 * integrates the absolute trailer heading ψ with ψ̇ = (v/d) sin(θ − ψ). The two
 * charts differ by the invertible map θ₁ = θ − ψ, which changes none of the rank
 * computations, so this module keeps the simulator's coordinate.
 */
export function carTrailer(L = 1, d = 1): CarTrailerFields {
  const drive = field(
    'g1',
    5,
    (q) => [Math.cos(q[3]), Math.sin(q[3]), 0, Math.tan(q[2]) / L, Math.sin(q[3] - q[4]) / d],
    (q) => {
      const J = zeros(5);
      J[0][3] = -Math.sin(q[3]);
      J[1][3] = Math.cos(q[3]);
      J[3][2] = 1 / Math.cos(q[2]) ** 2 / L;
      const c = Math.cos(q[3] - q[4]) / d;
      J[4][3] = c;
      J[4][4] = -c;
      return J;
    },
  );
  return { L, d, drive, steer: coordinateField(5, 2, 'g2'), hitchAngle: (q) => q[3] - q[4] };
}

/* -------------------------------------------------------------------------- */
/* Planar body with thrusters (PBWT), Choset Examples 12.1.2, 12.1.4, 12.1.8    */
/* -------------------------------------------------------------------------- */

export interface PbwtFields {
  d: number;
  aG: number;
  /** g₀ = [x₄, x₅, x₆, 0, −a_g, 0]ᵀ (gravity along −q₂; a_g = 0 for the horizontal plane). */
  drift: VectorField;
  /** g₁ = [0, 0, 0, cos x₃, sin x₃, 0]ᵀ — the thruster through the center of mass. */
  thrust1: VectorField;
  /** g₂ = [0, 0, 0, −sin x₃, cos x₃, −d]ᵀ — the offset thruster, which also spins the body. */
  thrust2: VectorField;
  /** Choset's six fields g₁…g₆ in closed form (Example 12.1.8), for the det = d⁴ check. */
  sixExact: (x: Vec) => Vec[];
}

/**
 * Unit mass and inertia, as in Choset; `m` and `I` scale the thrust fields for
 * Problem 14. Note the printed drift puts a_g at index 4: Choset writes
 * "+a_g" there and says gravity acts along −q₂; we take a_g ≥ 0 *downward*, so
 * the entry is −a_g and the two agree for a_g = 0, the only case §12.4 uses.
 */
export function pbwt(d = 1, aG = 0, m = 1, I = 1): PbwtFields {
  const drift = field(
    'g0',
    6,
    (x) => [x[3], x[4], x[5], 0, -aG, 0],
    () => {
      const J = zeros(6);
      J[0][3] = 1;
      J[1][4] = 1;
      J[2][5] = 1;
      return J;
    },
  );
  const thrust1 = field(
    'g1',
    6,
    (x) => [0, 0, 0, Math.cos(x[2]) / m, Math.sin(x[2]) / m, 0],
    (x) => {
      const J = zeros(6);
      J[3][2] = -Math.sin(x[2]) / m;
      J[4][2] = Math.cos(x[2]) / m;
      return J;
    },
  );
  const thrust2 = field(
    'g2',
    6,
    (x) => [0, 0, 0, -Math.sin(x[2]) / m, Math.cos(x[2]) / m, -d / I],
    (x) => {
      const J = zeros(6);
      J[3][2] = -Math.cos(x[2]) / m;
      J[4][2] = -Math.sin(x[2]) / m;
      return J;
    },
  );
  const sixExact = (x: Vec): Vec[] => {
    const c = Math.cos(x[2]);
    const s = Math.sin(x[2]);
    const w = x[5];
    return [
      [0, 0, 0, c, s, 0],
      [0, 0, 0, -s, c, -d],
      [-c, -s, 0, -w * s, w * c, 0],
      [s, -c, d, -w * c, -w * s, 0],
      [0, 0, 0, d * s, -d * c, 0],
      [-d * s, d * c, 0, d * w * c, d * w * s, 0],
    ];
  };
  return { d, aG, drift, thrust1, thrust2, sixExact };
}

/* -------------------------------------------------------------------------- */
/* Example 12.1.7 — a distribution that is not regular, and its nine leaves    */
/* -------------------------------------------------------------------------- */

export interface FoliationExample {
  /** g₁ = [x₁ cos x₃, x₂ sin x₃, 0]ᵀ, vanishing on the x₃-axis. */
  g1: VectorField;
  /** g₂ = ∂/∂x₃. */
  g2: VectorField;
  /** [g₁, g₂] = [x₁ sin x₃, −x₂ cos x₃, 0]ᵀ. */
  bracketExact: (x: Vec) => Vec;
  /** det[g₁ g₂ [g₁, g₂]] = x₁x₂. */
  detExact: (x: Vec) => number;
  /** Dimension of the integral manifold through x: 1 on the axis, 2 on a half-plane, 3 in a quadrant. */
  leafDimension: (x: Vec) => 1 | 2 | 3;
}

export function foliationExample(): FoliationExample {
  const g1 = field(
    'g1',
    3,
    (x) => [x[0] * Math.cos(x[2]), x[1] * Math.sin(x[2]), 0],
    (x) => [
      [Math.cos(x[2]), 0, -x[0] * Math.sin(x[2])],
      [0, Math.sin(x[2]), x[1] * Math.cos(x[2])],
      [0, 0, 0],
    ],
  );
  return {
    g1,
    g2: coordinateField(3, 2, 'g2'),
    bracketExact: (x) => [x[0] * Math.sin(x[2]), -x[1] * Math.cos(x[2]), 0],
    detExact: (x) => x[0] * x[1],
    leafDimension: (x) => {
      const a = Math.abs(x[0]) > 1e-12;
      const b = Math.abs(x[1]) > 1e-12;
      return a && b ? 3 : a || b ? 2 : 1;
    },
  };
}

/* -------------------------------------------------------------------------- */
/* §12.3's small examples with drift                                            */
/* -------------------------------------------------------------------------- */

/** ẋ = [x₂², 0]ᵀ + [0, 1]ᵀu: accessible and STLA everywhere, never controllable nor STLC. */
export function oneWayDrift(): { drift: VectorField; control: VectorField } {
  return {
    drift: field('g0', 2, (x) => [x[1] * x[1], 0], (x) => [
      [0, 2 * x[1]],
      [0, 0],
    ]),
    control: coordinateField(2, 1, 'g1'),
  };
}

/** The double integrator ẋ₁ = x₂, ẋ₂ = u: STLC only at rest (Figure 12.13). */
export function doubleIntegrator(): { drift: VectorField; control: VectorField } {
  return {
    drift: field('g0', 2, (x) => [x[1], 0], () => [
      [0, 1],
      [0, 0],
    ]),
    control: coordinateField(2, 1, 'g1'),
  };
}

/** Theorem 12.3.5's example: g₀ = ½[−x₂, x₁]ᵀ (closed orbits, WPPS), g₁ = ∂/∂x₁; det[g₁ [g₀, g₁]] = −½. */
export function wppsExample(): { drift: VectorField; control: VectorField; bracketExact: Vec } {
  return {
    drift: field('g0', 2, (x) => [-0.5 * x[1], 0.5 * x[0]], () => [
      [0, -0.5],
      [0.5, 0],
    ]),
    control: coordinateField(2, 0, 'g1'),
    bracketExact: [0, -0.5],
  };
}

/** Problem 8: linear fields Ax, Bx with [Ax, Bx] = (BA − AB)x under Choset's sign. */
export function linearField(A: Mat, name: string): VectorField {
  const n = A.length;
  return field(
    name,
    n,
    (x) => A.map((row) => row.reduce((s, a, j) => s + a * x[j], 0)),
    () => A.map((r) => r.slice()),
  );
}

/** Random polynomial field of degree ≤ 2 for the property checks (seeded by the caller). */
export function randomQuadraticField(n: number, name: string, draw: () => number): VectorField {
  const c0 = Array.from({ length: n }, draw);
  const c1: Mat = Array.from({ length: n }, () => Array.from({ length: n }, draw));
  const c2: number[][][] = Array.from({ length: n }, () => Array.from({ length: n }, () => Array.from({ length: n }, draw)));
  return field(
    name,
    n,
    (x) =>
      c0.map((c, i) => {
        let s = c;
        for (let j = 0; j < n; j++) {
          s += c1[i][j] * x[j];
          for (let k = 0; k < n; k++) s += c2[i][j][k] * x[j] * x[k];
        }
        return s;
      }),
    (x) =>
      c0.map((_, i) =>
        Array.from({ length: n }, (_, j) => {
          let s = c1[i][j];
          for (let k = 0; k < n; k++) s += (c2[i][j][k] + c2[i][k][j]) * x[k];
          return s;
        }),
      ),
  );
}
