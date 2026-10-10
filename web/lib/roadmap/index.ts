/**
 * `roadmap` — Chapter 8: Roadmaps I (Choset Ch. 5, §5.1–5.2).
 *
 * Owned artifacts later chapters import:
 *   - roadmap.ts   `Roadmap`: a graph embedded in Q_free that Chapter 6's A*
 *                  searches — the "get on, ride, get off" contract of Def. 5.0.2
 *   - morse.ts     the Morse definition box (index, Euler characteristic) that
 *                  Chapters 9–10 use without re-introducing it
 *   - retract.ts   gradient-ascent accessibility and the deformation retraction
 *                  (Chapter 9's GVG accesses the same way)
 *
 * The rest is this chapter's: the visibility graph by rotational sweep and by
 * brute force (visibility.ts), the exact planar GVD traced with Chapter 3's
 * curve tracer for small site sets (gvd.ts), and the grid GVD from Chapter 7's
 * brushfire ridges (grid.ts). Chapter 9 adds `gvg` as a submodule.
 */

export * from './roadmap';
export * from './visibility';
export * from './gvd';
export * from './retract';
export * from './grid';
export * from './morse';
export * from './examples';
