/**
 * `bugs` — Chapter 3: Bug algorithms, range-sensor geometry, and the curve tracer.
 *
 * Owned artifacts later chapters import:
 *   - scan.ts   ρ_R, D(x), ∇D from range data, intervals of continuity, O_i
 *   - trace.ts  traceCurve / traceStep: the App. D predictor–corrector
 */

export * from './geometry';
export * from './scan';
export * from './trace';
export * from './follow';
export * from './planner';
export * from './presets';
export { Bug1 } from './bug1';
export { Bug2 } from './bug2';
export { TangentBug, type Candidate, type TangentOptions, type TangentSensing } from './tangent';
