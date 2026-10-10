/**
 * `estimate` — the thin Part IV module: Choset's geometric-observer vocabulary
 * wrapped around the sister book's vendored filters (`../filters`, `../models`).
 *
 * Nothing here re-implements a Kalman filter. What it adds is what Choset's
 * Chapter 8 has and the sister does not: the fixed-gain Luenberger observer and
 * its error dynamics, the observability rank test, the §8.2.6 dead-reckoning
 * example, Choset's landmark Jacobians and χ² association, and the 2005 form of
 * Kalman SLAM.
 */
export * from './lti';
export * from './observer';
export * from './kf';
export * from './landmarks';
export * from './slam';
/*
 * Chapter 16 — Bayesian localization and mapping (Choset Ch. 9) as thin
 * wrappers over the vendored grid/particle localizers and occupancy grid, plus
 * the one artifact this book adds: replanning on a sampled belief.
 */
export * from './bayes';
export * from './replan';
