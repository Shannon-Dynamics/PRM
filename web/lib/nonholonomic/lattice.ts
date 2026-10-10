/**
 * The modern descendants of CAR GRID SEARCH: state lattices (Pivtoraiko,
 * Knepper & Kelly) and hybrid A* (Dolgov, Thrun, Montemerlo & Diebel). Both are
 * Algorithm 22 with one or two of three deltas:
 *
 *   1. lattice-consistent primitives — every motion starts *and ends* exactly on
 *      a lattice state (grid point × one of 8 headings), so the search graph is
 *      a true graph and A* runs on it unchanged (state lattice);
 *   2. one continuous state per cell — keep Alg. 22's free-running continuous
 *      configurations and its d³ occupancy grid, but let a cell hold the
 *      best-cost continuous state that reached it (hybrid A*);
 *   3. analytic expansions and a dual heuristic — every N expansions try a
 *      Reeds–Shepp "shot" straight to the goal and accept it if it is free, and
 *      order OPEN by max(RS length ignoring obstacles, 2-D distance ignoring
 *      kinematics) (hybrid A*).
 *
 * Search is Chapter 6's: a `KeyedHeap` on [f, h] with lazy deletion. The
 * primitive generator uses Chapter 21's own Dubins solver to connect lattice
 * states, so every primitive is a feasible full-lock-or-straight path of the
 * reduced car with radius ρ.
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';
import type { Hitch } from '../robots/hitch';
import type { Scene } from '../collide/scene';
import { FootprintChecker } from '../collide/collision';
import { KeyedHeap } from '../search/heap';
import { dubinsAll } from './dubins';
import { reedsSheppShortestPath, type RsSteer } from './reeds-shepp';
import { rsToSegments, sampleSegments, segmentsEnd, turningRadius, type CarSample, type CarSeg } from './car-path';

/** The Lot query of w21.3, the checks and the integration lab: reverse into bay 3 (both poses on every 2.5/k lattice). */
export const LOT_REVERSE_START: Pose2 = { x: 2.0, y: 1.0, theta: 0 };
export const LOT_REVERSE_GOAL: Pose2 = { x: 9.5, y: 8.5, theta: -Math.PI / 2 };

const H8 = 8;
const headingOf = (h: number) => normalizeAngle((h * Math.PI) / 4);
const mod = (a: number, n: number) => ((a % n) + n) % n;

/* -------------------------------------------------------------------------- */
/* Primitives                                                                  */
/* -------------------------------------------------------------------------- */

export interface Primitive {
  h0: number;
  di: number;
  dj: number;
  h1: number;
  segs: CarSeg[];
  /** Geometric length (m). */
  length: number;
  reverse: boolean;
  /** Body poses along the motion relative to the start's position (theta absolute). */
  samples: Pose2[];
  label: string;
}

export type PrimitiveSet = 'six' | 'sixteen';

interface Category {
  dh: number;
  kind: 'straight1' | 'straight2' | 'turn' | 'lane';
  side: number;
  maxTurn: number;
}

const SIX: Category[] = [
  { dh: 0, kind: 'straight1', side: 0, maxTurn: 0 },
  { dh: 1, kind: 'turn', side: 1, maxTurn: Math.PI / 4 },
  { dh: -1, kind: 'turn', side: -1, maxTurn: Math.PI / 4 },
];
const SIXTEEN: Category[] = [
  ...SIX,
  { dh: 0, kind: 'straight2', side: 0, maxTurn: 0 },
  { dh: 2, kind: 'turn', side: 1, maxTurn: Math.PI / 2 },
  { dh: -2, kind: 'turn', side: -1, maxTurn: Math.PI / 2 },
  { dh: 0, kind: 'lane', side: 1, maxTurn: Math.PI / 2 },
  { dh: 0, kind: 'lane', side: -1, maxTurn: Math.PI / 2 },
];

/**
 * Forward primitives from heading h0 to the cheapest lattice endpoint of each
 * category: straights along the heading (1 or 2 lattice steps), pure turns of
 * ±45° or ±90° (total turning no more than the heading change — no loops), and
 * lane changes (same heading, one lattice row to the side). Brute force over a
 * window of endpoints, Dubins for each; cached per (cell, ρ, set).
 */
function forwardPrimitives(h0: number, cell: number, rho: number, cats: Category[], sampleDs: number): Primitive[] {
  const th0 = headingOf(h0);
  const diag = h0 % 2 === 1;
  const ax = Math.round(Math.cos(th0) * (diag ? Math.SQRT2 : 1));
  const ay = Math.round(Math.sin(th0) * (diag ? Math.SQRT2 : 1));
  const W = Math.ceil((2.2 * rho) / cell) + 3;
  const out: Primitive[] = [];
  for (const cat of cats) {
    if (cat.kind === 'straight1' || cat.kind === 'straight2') {
      const k = cat.kind === 'straight1' ? 1 : 2;
      const len = k * cell * Math.hypot(ax, ay);
      const segs: CarSeg[] = [{ steer: 'S', len }];
      out.push(makePrim(h0, k * ax, k * ay, h0, segs, false, rho, sampleDs, cat.kind === 'straight1' ? 'S' : 'S2'));
      continue;
    }
    const h1 = mod(h0 + cat.dh, H8);
    const th1 = headingOf(h1);
    let best: { di: number; dj: number; segs: CarSeg[]; len: number } | null = null;
    for (let di = -W; di <= W; di++) {
      for (let dj = -W; dj <= W; dj++) {
        if (di === 0 && dj === 0) continue;
        const x = di * cell;
        const y = dj * cell;
        // Ahead of the start along its heading.
        const fwd = x * Math.cos(th0) + y * Math.sin(th0);
        if (fwd <= 1e-9) continue;
        const lat = -x * Math.sin(th0) + y * Math.cos(th0);
        if (cat.kind === 'lane') {
          // Exactly one lattice row to the requested side (a row is cell or cell/√2·2 on diagonals).
          const row = diag ? cell * Math.SQRT1_2 : cell;
          if (Math.abs(lat - cat.side * row) > 1e-9) continue;
        } else if (Math.sign(lat) !== cat.side && Math.abs(lat) > 1e-9) continue;
        const words = dubinsAll({ x: 0, y: 0, theta: th0 }, { x, y, theta: th1 }, rho);
        for (const w of words) {
          const turn = w.segs[0] * (w.word[0] === 'S' ? 0 : 1) + w.segs[2] * (w.word[2] === 'S' ? 0 : 1) + (w.word[1] === 'S' ? 0 : w.segs[1]);
          if (turn > cat.maxTurn + 1e-9) continue;
          if (cat.kind === 'turn' && w.word[1] !== 'S') continue;
          if (!best || w.length < best.len - 1e-12) {
            const segs: CarSeg[] = [];
            for (let i = 0; i < 3; i++) {
              const L = w.word[i] === 'S' ? w.segs[i] * rho : w.segs[i] * rho;
              if (L > 1e-9) segs.push({ steer: w.word[i] as RsSteer, len: L });
            }
            best = { di, dj, segs, len: w.length };
          }
          break; // words are sorted: the first admissible one is this endpoint's best
        }
      }
    }
    if (best) {
      const name = cat.kind === 'lane' ? (cat.side > 0 ? 'laneL' : 'laneR') : `${cat.side > 0 ? 'L' : 'R'}${Math.abs(cat.dh) * 45}`;
      out.push(makePrim(h0, best.di, best.dj, h1, best.segs, false, rho, sampleDs, name));
    }
  }
  return out;
}

function makePrim(h0: number, di: number, dj: number, h1: number, segs: CarSeg[], reverse: boolean, rho: number, ds: number, label: string): Primitive {
  const start: Pose2 = { x: 0, y: 0, theta: headingOf(h0) };
  const samples = sampleSegments(start, segs, rho, ds).map((s) => s.q);
  const length = segs.reduce((a, g) => a + Math.abs(g.len), 0);
  return { h0, di, dj, h1, segs, length, reverse, samples, label: reverse ? `${label}−` : `${label}+` };
}

const primCache = new Map<string, Primitive[][]>();

/**
 * Primitives per start heading. Reverse primitives reuse the forward ones of
 * the opposite heading: driving backward with heading θ traces the same curve
 * as driving forward with heading θ + π, with L and R swapped and the arc
 * lengths negated (the car rotates the same way while v changes sign).
 */
export function latticePrimitives(cell: number, rho: number, set: PrimitiveSet, sampleDs = 0.25): Primitive[][] {
  const key = `${cell.toFixed(6)}|${rho.toFixed(6)}|${set}|${sampleDs}`;
  const hit = primCache.get(key);
  if (hit) return hit;
  const cats = set === 'six' ? SIX : SIXTEEN;
  const fwd: Primitive[][] = [];
  for (let h = 0; h < H8; h++) fwd.push(forwardPrimitives(h, cell, rho, cats, sampleDs));
  const all: Primitive[][] = [];
  for (let h = 0; h < H8; h++) {
    const list = fwd[h].slice();
    for (const p of fwd[mod(h + 4, H8)]) {
      const segs = p.segs.map((g) => ({ steer: (g.steer === 'L' ? 'R' : g.steer === 'R' ? 'L' : 'S') as RsSteer, len: -g.len }));
      list.push(makePrim(h, p.di, p.dj, mod(p.h1 + 4, H8), segs, true, rho, sampleDs, p.label.slice(0, -1)));
    }
    all.push(list);
  }
  primCache.set(key, all);
  return all;
}

/* -------------------------------------------------------------------------- */
/* Heuristics                                                                  */
/* -------------------------------------------------------------------------- */

export type HeuristicKind = 'euclid' | 'rs' | 'dual';

/**
 * Obstacle-aware 2-D cost-to-go for the rear-axle point: Dijkstra on an
 * 8-connected grid from the goal, cells blocked within `inflate` of the scene.
 * The rear axle of a free car is at least min(rear overhang, half width) from
 * every obstacle, so inflating by that keeps the true path in the free cells;
 * octile steps overestimate Euclidean length by up to 8.2 %, so the table is
 * scaled by 1/1.0824 — the dual heuristic stays admissible.
 */
export class Holonomic2D {
  readonly nx: number;
  readonly ny: number;
  readonly dist: Float64Array;
  constructor(
    readonly scene: Scene,
    readonly goal: { x: number; y: number },
    readonly cell: number,
    inflate: number,
  ) {
    const b = scene.bounds;
    this.nx = Math.ceil((b.maxX - b.minX) / cell);
    this.ny = Math.ceil((b.maxY - b.minY) / cell);
    const n = this.nx * this.ny;
    const blocked = new Uint8Array(n);
    for (let j = 0; j < this.ny; j++) {
      for (let i = 0; i < this.nx; i++) {
        const x = b.minX + (i + 0.5) * cell;
        const y = b.minY + (j + 0.5) * cell;
        if (scene.pointInObstacle(x, y) || scene.distancePoint(x, y).dist < inflate) blocked[j * this.nx + i] = 1;
      }
    }
    this.dist = new Float64Array(n).fill(Infinity);
    const gi = Math.min(this.nx - 1, Math.max(0, Math.floor((goal.x - b.minX) / cell)));
    const gj = Math.min(this.ny - 1, Math.max(0, Math.floor((goal.y - b.minY) / cell)));
    const heap = new KeyedHeap<number>();
    const g0 = gj * this.nx + gi;
    this.dist[g0] = 0;
    heap.push([0], g0);
    const nb = [
      [1, 0, 1],
      [-1, 0, 1],
      [0, 1, 1],
      [0, -1, 1],
      [1, 1, Math.SQRT2],
      [1, -1, Math.SQRT2],
      [-1, 1, Math.SQRT2],
      [-1, -1, Math.SQRT2],
    ];
    while (heap.size) {
      const e = heap.pop()!;
      const c = e.value;
      if (e.key[0] > this.dist[c]) continue;
      const ci = c % this.nx;
      const cj = Math.floor(c / this.nx);
      for (const [di, dj, w] of nb) {
        const i = ci + di;
        const j = cj + dj;
        if (i < 0 || j < 0 || i >= this.nx || j >= this.ny) continue;
        const k = j * this.nx + i;
        if (blocked[k] && k !== g0) continue;
        const nd = this.dist[c] + w * cell;
        if (nd < this.dist[k]) {
          this.dist[k] = nd;
          heap.push([nd], k);
        }
      }
    }
  }

  at(x: number, y: number): number {
    const b = this.scene.bounds;
    const i = Math.min(this.nx - 1, Math.max(0, Math.floor((x - b.minX) / this.cell)));
    const j = Math.min(this.ny - 1, Math.max(0, Math.floor((y - b.minY) / this.cell)));
    const d = this.dist[j * this.nx + i];
    return Number.isFinite(d) ? d / 1.0824 : 1e6;
  }
}

function makeHeuristic(kind: HeuristicKind, goal: Pose2, rho: number, h2d: Holonomic2D | null): (q: Pose2) => number {
  if (kind === 'euclid') return (q) => Math.hypot(q.x - goal.x, q.y - goal.y);
  if (kind === 'rs') return (q) => reedsSheppShortestPath(q, goal, rho)?.length ?? 0;
  return (q) => Math.max(reedsSheppShortestPath(q, goal, rho)?.length ?? 0, h2d ? h2d.at(q.x, q.y) : 0);
}

/* -------------------------------------------------------------------------- */
/* Shared result type                                                          */
/* -------------------------------------------------------------------------- */

export interface Expansion {
  q: Pose2;
  parent: Pose2 | null;
  /** Body poses of the motion that reached q (for curved edges). */
  sweep: Pose2[];
}

export interface Shot {
  from: Pose2;
  samples: Pose2[];
  free: boolean;
}

export interface CarSearchResult {
  found: boolean;
  segs: CarSeg[];
  samples: CarSample[];
  /** Geometric length (m). */
  length: number;
  /** The weighted cost the search minimized. */
  cost: number;
  expansions: number;
  /** Every expanded state in order — the blue dots. */
  expanded: Expansion[];
  shots: Shot[];
  cusps: number;
}

export interface CarSearchOptions {
  car: Hitch;
  scene: Scene;
  start: Pose2;
  goal: Pose2;
  /** Lattice spacing / hybrid cell size (m). */
  cell: number;
  heuristic: HeuristicKind;
  /** Multiplier on reverse length in the cost. */
  reversePenalty?: number;
  maxExpansions?: number;
}

function cuspsOf(segs: readonly CarSeg[]): number {
  let c = 0;
  let prev = 0;
  for (const g of segs) {
    if (Math.abs(g.len) < 1e-12) continue;
    const s = Math.sign(g.len);
    if (prev !== 0 && s !== prev) c++;
    prev = s;
  }
  return c;
}

function mergeSegs(segs: readonly CarSeg[]): CarSeg[] {
  const out: CarSeg[] = [];
  for (const g of segs) {
    const last = out[out.length - 1];
    if (last && last.steer === g.steer && Math.sign(last.len) === Math.sign(g.len)) last.len += g.len;
    else out.push({ ...g });
  }
  return out;
}

/* -------------------------------------------------------------------------- */
/* State lattice: A* over (i, j, h)                                             */
/* -------------------------------------------------------------------------- */

export interface LatticeOptions extends CarSearchOptions {
  set: PrimitiveSet;
}

/**
 * A* over the lattice anchored at the start pose's position: lattice states
 * are (start + (i, j)·cell, h·45°). The goal must be a lattice state (its
 * offset a multiple of the cell, its heading a multiple of 45°) — the price of
 * delta 1, and the reason the chapter's query is chosen on the lattice.
 */
export function stateLatticeSearch(opts: LatticeOptions): CarSearchResult & { onLattice: boolean } {
  const { car, scene, start, goal, cell } = opts;
  const rho = turningRadius(car);
  const prims = latticePrimitives(cell, rho, opts.set, Math.min(0.25, cell));
  const checker = new FootprintChecker(scene, car);
  const gi = Math.round((goal.x - start.x) / cell);
  const gj = Math.round((goal.y - start.y) / cell);
  const gh = mod(Math.round(normalizeAngle(goal.theta) / (Math.PI / 4)), H8);
  const sh = mod(Math.round(normalizeAngle(start.theta) / (Math.PI / 4)), H8);
  const onLattice =
    Math.abs(gi * cell - (goal.x - start.x)) < 1e-9 &&
    Math.abs(gj * cell - (goal.y - start.y)) < 1e-9 &&
    Math.abs(normalizeAngle(gh * (Math.PI / 4) - goal.theta)) < 1e-9 &&
    Math.abs(normalizeAngle(sh * (Math.PI / 4) - start.theta)) < 1e-9;
  const b = scene.bounds;
  const iMin = Math.floor((b.minX - start.x) / cell) - 1;
  const iMax = Math.ceil((b.maxX - start.x) / cell) + 1;
  const jMin = Math.floor((b.minY - start.y) / cell) - 1;
  const jMax = Math.ceil((b.maxY - start.y) / cell) + 1;
  const NX = iMax - iMin + 1;
  const NY = jMax - jMin + 1;
  const key = (i: number, j: number, h: number) => ((h * NY + (j - jMin)) * NX + (i - iMin));
  const nStates = NX * NY * H8;
  const g = new Float64Array(nStates).fill(Infinity);
  const closed = new Uint8Array(nStates);
  const parent = new Int32Array(nStates).fill(-1);
  const viaPrim = new Int32Array(nStates).fill(-1);
  const goalPose = goal;
  const h2d = opts.heuristic === 'dual' ? new Holonomic2D(scene, goalPose, Math.max(0.25, cell), Math.min(car.p.rearOverhang, car.p.bodyWidth / 2)) : null;
  const hf = makeHeuristic(opts.heuristic, goalPose, rho, h2d);
  const pose = (i: number, j: number, h: number): Pose2 => ({ x: start.x + i * cell, y: start.y + j * cell, theta: headingOf(h) });
  const heap = new KeyedHeap<number>();
  const s0 = key(0, 0, sh);
  g[s0] = 0;
  heap.push([hf(pose(0, 0, sh)), 0], s0);
  const expanded: Expansion[] = [];
  const revPen = opts.reversePenalty ?? 1;
  const maxExp = opts.maxExpansions ?? 60000;
  const goalKey = key(gi, gj, gh);
  let found = false;
  while (heap.size && expanded.length < maxExp) {
    const e = heap.pop()!;
    const s = e.value;
    if (closed[s]) continue;
    closed[s] = 1;
    const h = Math.floor(s / (NX * NY));
    const rem = s - h * NX * NY;
    const j = Math.floor(rem / NX) + jMin;
    const i = (rem % NX) + iMin;
    const q = pose(i, j, h);
    const pk = parent[s];
    expanded.push({ q, parent: pk >= 0 ? decode(pk) : null, sweep: pk >= 0 ? prims[decodeH(pk)][viaPrim[s]].samples.map((p) => shift(p, decode(pk))) : [q] });
    if (s === goalKey) {
      found = true;
      break;
    }
    const plist = prims[h];
    for (let pi = 0; pi < plist.length; pi++) {
      const p = plist[pi];
      const ni = i + p.di;
      const nj = j + p.dj;
      if (ni < iMin || ni > iMax || nj < jMin || nj > jMax) continue;
      const nk = key(ni, nj, p.h1);
      if (closed[nk]) continue;
      const ng = g[s] + p.length * (p.reverse ? revPen : 1);
      if (ng >= g[nk]) continue;
      let free = true;
      for (const sp of p.samples) {
        if (!checker.isFree({ body: { x: q.x + sp.x, y: q.y + sp.y, theta: sp.theta }, trailer: null })) {
          free = false;
          break;
        }
      }
      if (!free) continue;
      g[nk] = ng;
      parent[nk] = s;
      viaPrim[nk] = pi;
      const np = pose(ni, nj, p.h1);
      const hv = hf(np);
      heap.push([ng + hv, hv], nk);
    }
  }
  function decodeH(k: number): number {
    return Math.floor(k / (NX * NY));
  }
  function decode(k: number): Pose2 {
    const h = Math.floor(k / (NX * NY));
    const rem = k - h * NX * NY;
    return pose((rem % NX) + iMin, Math.floor(rem / NX) + jMin, h);
  }
  function shift(p: Pose2, base: Pose2): Pose2 {
    return { x: base.x + p.x, y: base.y + p.y, theta: p.theta };
  }
  if (!found) return { found: false, segs: [], samples: [], length: 0, cost: Infinity, expansions: expanded.length, expanded, shots: [], cusps: 0, onLattice };
  const chain: number[] = [];
  for (let s = goalKey; s !== s0; s = parent[s]) chain.push(s);
  chain.reverse();
  const segs: CarSeg[] = [];
  let prevKey = s0;
  for (const s of chain) {
    const p = prims[decodeH(prevKey)][viaPrim[s]];
    segs.push(...p.segs);
    prevKey = s;
  }
  const merged = mergeSegs(segs);
  const startPose: Pose2 = { x: start.x, y: start.y, theta: headingOf(sh) };
  const samples = sampleSegments(startPose, merged, rho, 0.1);
  const length = merged.reduce((a, g2) => a + Math.abs(g2.len), 0);
  return { found: true, segs: merged, samples, length, cost: g[goalKey], expansions: expanded.length, expanded, shots: [], cusps: cuspsOf(merged), onLattice };
}

/* -------------------------------------------------------------------------- */
/* Hybrid A*                                                                    */
/* -------------------------------------------------------------------------- */

export interface HybridOptions extends CarSearchOptions {
  /** Try a Reeds–Shepp shot every N expansions (1 = every expansion). */
  shotEvery: number;
  /** Heading bins of the cell grid (default 72, Dolgov et al.'s 5°). */
  headingBins?: number;
  /** Arc length of one primitive; default just long enough to leave a cell and a heading bin. */
  stepLen?: number;
}

interface HNode {
  q: Pose2;
  g: number;
  parent: number;
  seg: CarSeg | null;
  sweep: Pose2[];
}

/**
 * Algorithm 22 with continuous states (one per cell, the cheapest), A* order
 * under the chosen heuristic, the six actions L±, S±, R±, and analytic
 * Reeds–Shepp shots. The search ends only through a free shot, so the path
 * ends *exactly* at the goal — the delta that made hybrid A* drive real cars.
 */
export function hybridAStar(opts: HybridOptions): CarSearchResult {
  const { car, scene, start, goal, cell } = opts;
  const rho = turningRadius(car);
  const checker = new FootprintChecker(scene, car);
  const HB = opts.headingBins ?? 72;
  const b = scene.bounds;
  const nx = Math.ceil((b.maxX - b.minX) / cell);
  const ny = Math.ceil((b.maxY - b.minY) / cell);
  const cellOf = (q: Pose2) => {
    const i = Math.min(nx - 1, Math.max(0, Math.floor((q.x - b.minX) / cell)));
    const j = Math.min(ny - 1, Math.max(0, Math.floor((q.y - b.minY) / cell)));
    let th = q.theta % (2 * Math.PI);
    if (th < 0) th += 2 * Math.PI;
    const k = Math.min(HB - 1, Math.floor((th / (2 * Math.PI)) * HB));
    return (k * ny + j) * nx + i;
  };
  const N = nx * ny * HB;
  const bestG = new Float64Array(N).fill(Infinity);
  const closed = new Uint8Array(N);
  // Long enough to leave the cell *and*, at full lock, the heading bin — otherwise a turning child
  // shares its cell with a straight sibling of equal cost and is pruned, and the search can never turn.
  const step = opts.stepLen ?? 1.05 * Math.max(Math.SQRT2 * cell, rho * ((2 * Math.PI) / HB));
  const revPen = opts.reversePenalty ?? 1;
  const h2d = opts.heuristic === 'dual' ? new Holonomic2D(scene, goal, Math.max(0.25, cell), Math.min(car.p.rearOverhang, car.p.bodyWidth / 2)) : null;
  const hf = makeHeuristic(opts.heuristic, goal, rho, h2d);
  const nodes: HNode[] = [{ q: start, g: 0, parent: -1, seg: null, sweep: [start] }];
  const heap = new KeyedHeap<number>();
  heap.push([hf(start), 0], 0);
  bestG[cellOf(start)] = 0;
  const expanded: Expansion[] = [];
  const shots: Shot[] = [];
  const maxExp = opts.maxExpansions ?? 40000;
  const acts: { steer: RsSteer; dir: number }[] = [
    { steer: 'L', dir: 1 },
    { steer: 'S', dir: 1 },
    { steer: 'R', dir: 1 },
    { steer: 'L', dir: -1 },
    { steer: 'S', dir: -1 },
    { steer: 'R', dir: -1 },
  ];
  let goalNode = -1;
  let shotSegs: CarSeg[] = [];
  while (heap.size && expanded.length < maxExp) {
    const e = heap.pop()!;
    const id = e.value;
    const n = nodes[id];
    const c = cellOf(n.q);
    if (closed[c] && id !== 0) continue;
    closed[c] = 1;
    expanded.push({ q: n.q, parent: n.parent >= 0 ? nodes[n.parent].q : null, sweep: n.sweep });
    // Analytic expansion: a Reeds–Shepp shot to the goal.
    if ((expanded.length - 1) % Math.max(1, opts.shotEvery) === 0) {
      const rs = reedsSheppShortestPath(n.q, goal, rho);
      if (rs) {
        const segs = rsToSegments(rs);
        const samp = sampleSegments(n.q, segs, rho, Math.min(0.2, cell));
        let free = true;
        for (const s of samp) {
          if (!checker.isFree({ body: s.q, trailer: null })) {
            free = false;
            break;
          }
        }
        shots.push({ from: n.q, samples: samp.map((s) => s.q), free });
        if (free) {
          goalNode = id;
          shotSegs = segs;
          break;
        }
      }
    }
    for (const a of acts) {
      const len = a.dir * step;
      const sub = 3;
      const sweep: Pose2[] = [n.q];
      let free = true;
      let q = n.q;
      for (let k = 1; k <= sub; k++) {
        q = segmentsEnd(n.q, [{ steer: a.steer, len: (len * k) / sub }], rho);
        sweep.push(q);
        if (!checker.isFree({ body: q, trailer: null })) {
          free = false;
          break;
        }
      }
      if (!free) continue;
      const cc = cellOf(q);
      if (closed[cc]) continue;
      const ng = n.g + step * (a.dir < 0 ? revPen : 1);
      if (ng >= bestG[cc]) continue;
      bestG[cc] = ng;
      const nid = nodes.length;
      nodes.push({ q, g: ng, parent: id, seg: { steer: a.steer, len }, sweep });
      const hv = hf(q);
      heap.push([ng + hv, hv], nid);
    }
  }
  if (goalNode < 0) return { found: false, segs: [], samples: [], length: 0, cost: Infinity, expansions: expanded.length, expanded, shots, cusps: 0 };
  const chain: CarSeg[] = [];
  for (let id = goalNode; nodes[id].parent >= 0; id = nodes[id].parent) chain.push(nodes[id].seg!);
  chain.reverse();
  const segs = mergeSegs([...chain, ...shotSegs]);
  const samples = sampleSegments(start, segs, rho, 0.1);
  const length = segs.reduce((a, g2) => a + Math.abs(g2.len), 0);
  const cost = segs.reduce((a, g2) => a + Math.abs(g2.len) * (g2.len < 0 ? revPen : 1), 0);
  return { found: true, segs, samples, length, cost, expansions: expanded.length, expanded, shots, cusps: cuspsOf(segs) };
}
