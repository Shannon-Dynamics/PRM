/**
 * Chapter 20's numeric micro-example, the contract between the prose, the
 * widgets and the checks: the unicycle bracket at x₃ = π/6, the exact four-flow
 * residue at ε = 0.1 against ε²[g₁, g₂], det[g₁ g₂ [g₁, g₂]] = 1; Hitch's car at
 * q = 0 with L = 1: the ladder (2, 3, 4), det[g₁ g₂ g₃ g₄] = −1, STLC by
 * symmetry; the trailer's (2, 3, 4, 5). Every number printed in the chapter
 * is produced here and pinned in `__checks_ch20__.ts`.
 */

import type { Vec } from '../prob/linalg';
import { bracketAt, detColumns, fourFlowLoop, lieBracket } from './field';
import { filtration, larc, type ControlAffine, type Verdict } from './filtration';
import { car, carTrailer, unicycle } from './systems';

export interface UnicycleMicro {
  x: Vec;
  g1: Vec;
  bracket: Vec;
  bracketExact: Vec;
  eps: number;
  residue: Vec;
  residueExact: Vec;
  predicted: Vec;
  remainder: number;
  det: number;
}

/** The unicycle at x₃ = π/6 and ε = 0.1 (design doc §3). */
export function unicycleMicro(x3 = Math.PI / 6, eps = 0.1): UnicycleMicro {
  const u = unicycle();
  const x = [0, 0, x3];
  const g1 = u.drive.eval(x);
  const bracket = bracketAt(u.drive, u.spin, x);
  const loop = fourFlowLoop(u.drive, u.spin, x, eps, 64);
  const det = detColumns([g1, u.spin.eval(x), bracket]);
  return {
    x,
    g1,
    bracket,
    bracketExact: u.bracketExact(x),
    eps,
    residue: loop.residue,
    residueExact: u.loopExact(x, eps),
    predicted: loop.predicted,
    remainder: loop.remainder,
    det,
  };
}

export interface CarMicro {
  q: Vec;
  columns: Vec[];
  det: number;
  detExact: number;
  dims: number[];
  verdict: Verdict;
}

/** Hitch's car (eq. 12.34) with L = 1 at q = 0: columns e₁, e₃, −e₄, e₂; det = −1; dims (2, 3, 4). */
export function carMicro(L = 1, q: Vec = [0, 0, 0, 0]): CarMicro {
  const c = car(L);
  const g3 = lieBracket(c.drive, c.steer, 'g3');
  const g4 = lieBracket(c.drive, g3, 'g4');
  const columns = [c.drive.eval(q), c.steer.eval(q), g3.eval(q), g4.eval(q)];
  const sys: ControlAffine = { name: 'car', n: 4, drift: null, controls: [c.drive, c.steer], controlSet: 'U±' };
  const r = larc(sys, q, 4);
  return { q, columns, det: detColumns(columns), detExact: c.detExact(q), dims: r.filtration.dims, verdict: r.verdict };
}

/** The car with one trailer (L = d = 1) at q = 0, and at a hitch angle of π/2: the ladder (2, 3, 4, 5) both times. */
export function trailerMicro(L = 1, d = 1, hitch = 0): { q: Vec; dims: number[]; rank: number } {
  const t = carTrailer(L, d);
  const q = [0, 0, 0, 0, -hitch];
  const f = filtration([t.drive, t.steer], q, { maxDeg: 4 });
  return { q, dims: f.dims, rank: f.rank };
}

export function carSystem(L = 1): ControlAffine {
  const c = car(L);
  return { name: 'car', n: 4, drift: null, controls: [c.drive, c.steer], controlSet: 'U±' };
}

export function unicycleSystem(controlSet: 'U±' | 'U+' = 'U±'): ControlAffine {
  const u = unicycle();
  return { name: 'unicycle', n: 3, drift: null, controls: [u.drive, u.spin], controlSet };
}

export function trailerSystem(L = 1, d = 1): ControlAffine {
  const t = carTrailer(L, d);
  return { name: 'car+trailer', n: 5, drift: null, controls: [t.drive, t.steer], controlSet: 'U±' };
}
