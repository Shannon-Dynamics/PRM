/**
 * `dynamics` — Lagrangian dynamics of Reach (Chapter 17): the standard form
 * M q̈ + C q̇ + g = u, Christoffel symbols, Pfaffian constraints, rigid-body
 * rotation, and the integrators. Imported by Chapters 18–21 and 23.
 */
export * from './model';
export * from './lagrange';
export * from './planar-arm';
export * from './reach2r';
export * from './constraint';
export * from './rigid';
export * from './integrate';
