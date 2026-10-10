# Principles of Robot Motion via Rust

An interactive book on robot motion planning, taught the **FCP way** — rigorous mathematical
**F**oundations, **C**onceptual understanding through simulations you can manipulate, and
**P**ractical implementations in Rust.

Twenty-three chapters take three robots — **Rusty** the rover, **Reach** the planar arm, and
**Hitch** the car with a trailer — from a bug crawling around an obstacle to a full planning
stack: configuration space and its topology, graph search, potential functions, visibility and
Voronoi roadmaps, cell decompositions and coverage, probabilistic roadmaps and tree planners,
asymptotically optimal planning, multi-robot and manipulation planning, estimation as a planner
needs it, Lagrangian dynamics, time-optimal trajectories, trajectory optimization and MPC,
controllability and steering of nonholonomic systems, learning in the planning loop, and a
capstone that runs all of it in the browser.

The book modernizes Choset et al., *Principles of Robot Motion: Theory, Algorithms, and
Implementations* (MIT Press, 2005), and is the sister volume of
[*Probabilistic Robotics via Rust*](../ProbabilisticRoboticRust/), with which it shares its robot,
its simulator, and its publishing architecture.

## Running the book while it is being written

```sh
export PATH="$HOME/.local/node/bin:$PATH"   # Node 24 (see web/README.md to install it)
cd web && npm install
npm run build        # builds whatever chapters exist; unwritten ones show as "in preparation"
npm run preview      # serve the built book (compressed, like GitHub Pages)
npm run check        # numerical invariants of every landed algorithm module
ALLOW_MISSING=1 npm run check:book   # book consistency, tolerating chapters not yet written
```

`npm run build` regenerates the sidebar from the chapter files present (`scripts/sync-meta.mjs`),
so a partially written book always builds. The strict `npm run verify` is for the finished book.

## Status: implementation in progress

This repository holds the **design documents** the book is built from, the web edition under
`web/` (chapters land one at a time), and the print pipeline under `pr-latex/`:

```
TOC.md                the book's contract: structure, cast, notation, color code, crate stack,
                      shared-artifact ownership, Choset→chapter source map
Chapter-01.md … 23    per-chapter design docs (storyline, maths, widget manifest, Rust plan,
                      exercises, modernization notes)
CLAUDE.md             how to develop this project
```

- `web/` — the Next.js static site (chapters as MDX, widgets as React client components, a
  TypeScript port of the Rust algorithms powering the simulations), forked from the sister book.
- `pr-latex/` — the print edition generated from the web edition; builds once chapters exist.

## Two editions, one source (planned)

| | |
|---|---|
| **Web** — `web/` | Next.js static site; every chapter carries live simulations whose algorithms are the ones the chapter teaches. |
| **Print** — `pr-latex/` | PDF generated from the web edition, so the two cannot drift apart. |

## The simulations are the algorithms

As in the sister book, `web/lib/` will be a TypeScript port of the Rust the book teaches, pinned by
numerical invariants the mathematics guarantees: Dubins words integrate to their endpoints, the
star algorithm equals a brute-force Minkowski sum, A\* with an admissible heuristic matches
Dijkstra's cost, a time-optimal phase-plane trajectory touches the velocity-limit curve exactly at
its switching points.

## Reference material

The baseline text is Choset et al. (2005); the sister book's six baselines and a modernization set
(Karaman & Frazzoli, Gammell et al., Janson et al., Ratliff/Zucker, Schulman et al., Pham & Pham,
Dolgov et al., Koenig & Likhachev, Sharon et al., Ichter et al., Janner et al., …) are listed with
verification status in `TOC.md §4`.

The PDF is **not committed**: it is a third-party copyrighted work. Place it, and its extracted
text (`choset.txt`), in `Resource/` if you want the tooling that reads it. Nothing in the build
requires it.
