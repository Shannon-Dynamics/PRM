/**
 * `potential` — Chapter 7: Potential Functions (Choset Ch. 4).
 *
 * Owned artifacts later chapters import:
 *   - brushfire.ts   `brushfire` / `wavefront` / `ridgeCells`: THE grid distance
 *                    transform (Ch. 8 §5.2.5 grid GVD, Ch. 10 §6.2.4)
 *   - field.ts       `Potential`, `Attractive`, `Repulsive`, `Additive`, Hessian
 *                    and critical-point classification (Ch. 19's CHOMP reads a
 *                    distance field through the same interface)
 *
 * The rest is this chapter's: distance queries in the plane (geometry, range
 * scan, grid), gradient descent (C Alg. 4), navigation functions on sphere and
 * star worlds (§4.6), and the Jᵀ lift of workspace forces to Reach's joint
 * torques (§4.7).
 */

export * from './field';
export * from './planar';
export * from './descent';
export * from './brushfire';
export * from './navigation';
export * from './lifted';
