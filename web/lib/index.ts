/**
 * Public surface of the book's algorithm library.
 *
 * Everything a chapter widget needs, in dependency order: randomness and linear
 * algebra at the bottom, then geometry, then the worlds and robots, then the
 * planners. Import from `@/lib` for the whole surface, or from the individual
 * module when a widget only needs one piece.
 *
 * `prob/`, `geom/`, `sim/`, `mapping/edt` are ported unchanged from the sister
 * book (*Probabilistic Robotics via Rust*); `filters/`, `models/`, `localize/`
 * are vendored for Part IV only. Everything else is this book's own.
 */

// Foundations (ported)
export * from './prob/rng';
export * from './prob/linalg';
export * from './geom/se2';

// Worlds and the rover (ported)
export * from './sim/world';
export * from './mapping/edt';

// This book's modules register themselves here as chapters land:
// robots/ collide/ cspace/ manifold/ search/ potential/ roadmap/ cells/
// sampling/ multi/ estimate/ dynamics/ trajectory/ trajopt/ nonholonomic/ learn/ capstone/

// @chapter-exports-start
export * as bugs from './bugs';
export * as capstone from './capstone';
export * as cells from './cells';
export * as collide from './collide';
export * as cspace from './cspace';
export * as dynamics from './dynamics';
export * as estimate from './estimate';
export * as learn from './learn';
export * as manifold from './manifold';
export * as multi from './multi';
export * as nonholonomic from './nonholonomic';
export * as potential from './potential';
export * as roadmap from './roadmap';
export * as robots from './robots';
export * as sampling from './sampling';
export * as search from './search';
export * as trajectory from './trajectory';
export * as trajopt from './trajopt';
// @chapter-exports-end

// Numerical self-verification
export * from './__checks__';
