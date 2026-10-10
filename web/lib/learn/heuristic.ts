/**
 * A learned cost-to-go for Chapter 6's A*, and the audit that keeps it honest.
 *
 * The net does not predict h* outright: it predicts log(h* ÷ h_oct), the factor
 * by which obstacles stretch the empty-torus octile distance. So the estimate
 * is ĥ = h_oct · exp(ŷ) ≥ h_oct whenever ŷ ≥ 0 — *more informed* than the
 * admissible baseline by construction, and *inadmissible* wherever the net
 * guesses high. How high is a number, not a hope: `measureEpsilon` returns the
 * largest ĥ/h* − 1 on validation worlds, and Derivation 3 turns that number
 * into the exact bound (1 + ε) C* on the path A* can return.
 *
 * `HeuristicNet.h` is a closure `(cell) => number`, which is all Chapter 6's
 * `astar` ever wanted from a heuristic; the engine is untouched.
 */

import type { Rng } from '../prob/rng';
import { BestFirst, PRIORITY_ASTAR, astar, type Trace } from '../search/best-first';
import { rasterTorusCObstacle, type TorusLab } from '../sampling/worlds';
import type { Mlp } from './mlp';
import { TorusGridGraph, costToGo, torusOctile } from './torus-grid';
import { ENC_DIM, encodeAngles, type EnvEncoding, type WorldCase } from './worlds';

/** Raster side for the heuristic's lattice: 24 × 24 cells of 15°. */
export const HEUR_RASTER_N = 24;
export const HEURISTIC_IN = ENC_DIM + 8;
export const HEURISTIC_SIZES: readonly number[] = [HEURISTIC_IN, 48, 1];

export function heuristicFeatures(enc: EnvEncoding, g: TorusGridGraph, cell: number, goal: number): Float64Array {
  const x = new Float64Array(HEURISTIC_IN);
  x.set(enc, 0);
  x.set(encodeAngles(g.center(cell)), ENC_DIM);
  x.set(encodeAngles(g.center(goal)), ENC_DIM + 4);
  return x;
}

/** The lattice of a world case at the heuristic's resolution. */
export function latticeOf(lab: TorusLab, n = HEUR_RASTER_N): TorusGridGraph {
  return new TorusGridGraph(rasterTorusCObstacle(lab, n));
}

export type HeuristicKind = 'octile' | 'learned' | 'learned-deflated';

export interface HeuristicNetOptions {
  /** Multiplies ĥ — the headline knob of w22.2. 1 is the net as trained. */
  scale?: number;
  /** Divide ĥ by (1 + ε_val): the "deflated" variant, admissible on the validation set and nowhere else for certain. */
  deflate?: number;
}

export class HeuristicNet {
  /** ĥ for every cell at scale 1, before deflation (obstacle cells hold 0). */
  readonly table: Float64Array;
  readonly octile: (cell: number) => number;
  readonly scale: number;
  readonly deflate: number;
  readonly inferenceMs: number;

  constructor(
    readonly net: Mlp,
    readonly enc: EnvEncoding,
    readonly graph: TorusGridGraph,
    readonly goal: number,
    opts: HeuristicNetOptions = {},
  ) {
    this.scale = opts.scale ?? 1;
    this.deflate = opts.deflate ?? 0;
    this.octile = torusOctile(graph, goal);
    const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
    this.table = new Float64Array(graph.size);
    for (let c = 0; c < graph.size; c++) {
      if (!graph.isFree(c)) continue;
      const base = this.octile(c);
      if (base === 0) continue;
      const y = net.forward(heuristicFeatures(enc, graph, c, goal))[0];
      // exp(ŷ) ≥ 1 for ŷ ≥ 0; clamp the exponent so a wild guess cannot overflow.
      this.table[c] = base * Math.exp(Math.min(3, Math.max(-1, y)));
    }
    this.inferenceMs = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;
  }

  /** The heuristic closure Chapter 6's A* takes. */
  readonly h = (cell: number): number => (this.scale * this.table[cell]) / (1 + this.deflate);
}

/* -------------------------------------------------------------------------- */
/* Measuring ε                                                                 */
/* -------------------------------------------------------------------------- */

export interface EpsilonReport {
  /** max over audited nodes of ĥ/h* − 1, floored at 0. */
  epsilon: number;
  /** Nodes with ĥ > h*. */
  violations: number;
  audited: number;
  /** Mean ĥ/h* — how informed the net is on average (1 = exact). */
  meanRatio: number;
  /** Mean h_oct/h* for comparison — how loose the admissible baseline is. */
  meanOctileRatio: number;
  worst: { seed: number; cell: number; goal: number; hStar: number; hHat: number } | null;
}

/**
 * Audit ĥ against backward-Dijkstra labels on `cases`, `goalsPerWorld` random
 * goals each. Every reachable free cell with h* > 0 is checked — the whole
 * lattice, not a sample — so the ε returned is the true maximum on these worlds.
 */
export function measureEpsilon(net: Mlp, cases: readonly WorldCase[], goalsPerWorld: number, rng: Rng, scale = 1): EpsilonReport {
  let epsilon = 0;
  let violations = 0;
  let audited = 0;
  let ratioSum = 0;
  let octSum = 0;
  let worst: EpsilonReport['worst'] = null;
  for (const wc of cases) {
    const g = latticeOf(wc.lab);
    for (let k = 0; k < goalsPerWorld; k++) {
      const goal = randomFreeCell(g, rng);
      if (goal < 0) continue;
      const hStar = costToGo(g, goal);
      const hn = new HeuristicNet(net, wc.enc, g, goal, { scale });
      for (let c = 0; c < g.size; c++) {
        const hs = hStar[c];
        if (!Number.isFinite(hs) || hs <= 0) continue;
        const hh = hn.h(c);
        audited++;
        ratioSum += hh / hs;
        octSum += hn.octile(c) / hs;
        if (hh > hs + 1e-9) {
          violations++;
          const e = hh / hs - 1;
          if (e > epsilon) {
            epsilon = e;
            worst = { seed: wc.seed, cell: c, goal, hStar: hs, hHat: hh };
          }
        }
      }
    }
  }
  return { epsilon, violations, audited, meanRatio: audited ? ratioSum / audited : 1, meanOctileRatio: audited ? octSum / audited : 1, worst };
}

export function randomFreeCell(g: TorusGridGraph, rng: Rng): number {
  for (let t = 0; t < 200; t++) {
    const c = Math.floor(rng.next() * g.size);
    if (g.isFree(c)) return c;
  }
  return -1;
}

/* -------------------------------------------------------------------------- */
/* A* with a pluggable heuristic, and what we read off it                       */
/* -------------------------------------------------------------------------- */

export interface SearchReport {
  found: boolean;
  cost: number;
  expansions: number;
  reopened: number;
  path: number[];
}

/** Chapter 6's A* (with reopening) on the torus lattice. */
export function aStarEps(g: TorusGridGraph, start: number, goal: number, h: (cell: number) => number): SearchReport {
  const t: Trace<number> = astar(g, start, goal, h);
  return { found: t.status === 'found', cost: t.cost, expansions: t.expansions, reopened: t.reopened, path: t.path };
}

/** The engine itself, for widgets that animate the expansions. */
export function aStarEngine(g: TorusGridGraph, start: number, goal: number, h: (cell: number) => number): BestFirst<number> {
  return new BestFirst<number>(g, start, goal, h, PRIORITY_ASTAR);
}

/**
 * Derivation 3's guarantee as a predicate: with ĥ ≤ (1 + ε) h* everywhere,
 * the returned cost may not exceed (1 + ε) C*.
 */
export function withinEpsBound(cost: number, cStar: number, epsilon: number, tol = 1e-9): boolean {
  return cost <= (1 + epsilon) * cStar + tol;
}

/* -------------------------------------------------------------------------- */
/* The §3(b) micro-example: three nodes on a chain                              */
/* -------------------------------------------------------------------------- */

export interface AdmissibilityAudit {
  ratios: number[];
  violated: string[];
  /** Nodes where ĥ(v) > c(v, v′) + ĥ(v′) along the chain. */
  inconsistent: string[];
  epsilon: number;
  /** (1 + ε) · h*(first node): the F3 bound on any path from the chain's head. */
  bound: number;
}

/**
 * Audit ĥ against h* on a chain v₀ → v₁ → … → goal with the given edge costs
 * (the last cost leads into the goal, where both heuristics are 0).
 */
export function admissibilityAudit(labels: readonly string[], hStar: readonly number[], hHat: readonly number[], chainCosts: readonly number[]): AdmissibilityAudit {
  const ratios = hStar.map((hs, i) => hHat[i] / hs);
  const violated = labels.filter((_, i) => hHat[i] > hStar[i] + 1e-12);
  const inconsistent: string[] = [];
  for (let i = 0; i < labels.length; i++) {
    const next = i + 1 < labels.length ? hHat[i + 1] : 0;
    if (hHat[i] > chainCosts[i] + next + 1e-12) inconsistent.push(labels[i]);
  }
  const epsilon = Math.max(0, Math.max(...ratios) - 1);
  return { ratios, violated, inconsistent, epsilon, bound: (1 + epsilon) * hStar[0] };
}

export const THREE_NODE = {
  labels: ['a', 'b', 'c'] as const,
  hStar: [5.0, 3.0, 1.0],
  hHat: [4.6, 3.4, 0.8],
  /** a →2→ b →2→ c →1→ goal */
  chainCosts: [2, 2, 1],
};

export const threeNodeExample = () => admissibilityAudit(THREE_NODE.labels, THREE_NODE.hStar, THREE_NODE.hHat, THREE_NODE.chainCosts);
