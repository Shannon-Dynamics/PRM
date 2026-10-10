/**
 * `cells` — Chapter 10: Cell Decompositions and Coverage (Choset Ch. 6).
 *
 * Owned artifacts later chapters import:
 *   - world.ts     `CellWorld`: a polygonal free space (boundary + holes), the
 *                  general-position shear, Δ and P_total
 *   - sweep.ts     the vertical sweep: trapezoidal and boustrophedon
 *                  decompositions, the list L, Choset's four event types,
 *                  slab point location (the exact baseline Chapter 11's PRM
 *                  is measured against)
 *   - plan.ts      the adjacency graph as a Chapter 6 `Graph`, A*, and
 *                  midpoint–centroid chaining (Chapter 14's discrete layer)
 *   - reeb.ts      `ReebGraph` and Euler's count N_ce = N_cp + N_ob − 1
 *   - coverage.ts  exhaustive walk, lawnmower laps, measured coverage, the
 *                  missed-area and path-length bounds
 *   - online.ts    sensor-based coverage: the cycle algorithm on Chapter 3's
 *                  offset follower and range ring, growing the Reeb graph
 *
 * The rest is this chapter's: Morse decompositions for variable slices on a
 * raster (slices.ts), the pursuit/evasion coda (pursuit.ts), the lab (lab.ts)
 * and the worlds (examples.ts).
 */

export * from './world';
export * from './sweep';
export * from './plan';
export * from './reeb';
export * from './slices';
export * from './coverage';
export * from './online';
export * from './pursuit';
export * from './examples';
export * from './lab';
