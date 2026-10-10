/**
 * `cspace` — Configuration Space I (Chapter 4): C-obstacles made computable.
 *
 * The rasterizer for any `Collision<Q>` over a 2-D chart (§3.2.2), the disc's
 * inflation (§3.2.1), the star algorithm and SE(2) slices (App. F.3–F.4), the
 * contact test behind the collision box (App. F.2), GJK (App. F.5), and
 * Reach's Jacobian with its singularities (§3.8). Chapters 7, 8, 10, 17–18
 * and 21 import from here and never reimplement these.
 */
export * from './polygon';
export * from './star';
export * from './gjk';
export * from './raster';
export * from './jacobian';
export * from './examples';
