# CLAUDE.md — Principles of Robot Motion via Rust

Guidance for Claude Code (and human contributors) when developing this project.

## What this project is

An **interactive web book**, *Principles of Robot Motion via Rust*, taught the **FCP way** — every
chapter delivers **F**oundation (full mathematical formalism and derivations), **C**onceptual
(interactive visual dashboards, simulations, and animations in the page), and **P**ractical
(idiomatic Rust with the best current crates). The book modernizes Choset, Lynch, Hutchinson,
Kantor, Burgard, Kavraki & Thrun's *Principles of Robot Motion* (MIT Press, 2005) — configuration
space, bug algorithms, potentials, roadmaps, cell decompositions, sampling-based planning,
dynamics, trajectory planning, nonholonomic systems — and extends it with asymptotically optimal
planners, trajectory optimization and MPC, multi-robot and manipulation planning, learning in the
planning loop, and a capstone planning stack.

It is the **sister volume** of *Probabilistic Robotics via Rust* (`../ProbabilisticRoboticRust/`).
Same method, same publishing architecture, same robot Rusty. Estimation is treated here only as
far as a planner needs it (Part IV) and cross-linked to the sister book for depth.

**The core promise: the Rust code printed in the text is the same code that powers the in-page
simulations.** Never break this — no demo may fork away from the library code it claims to
demonstrate.

## Current state & source-of-truth documents

| File | Role |
|---|---|
| `TOC.md` | **The contract.** Vision, cast and worlds, 23-chapter / 6-part / 6-appendix structure, book-wide notation table, planning color code, shared-artifact ownership table, crate stack, publishing architecture, and the complete Choset-section → chapter source map. Nothing may contradict it; change `TOC.md` *first*, then dependents. |
| `Chapter-01.md` … `Chapter-23.md` | Per-chapter design docs: storyline, F/C/P plan, widget manifests (ids `w<chapter>.<k>`), Rust module plans, exercises, modernization notes. These are the specs to implement. |
| `Resource/choset.txt` (ignored, local) | Extracted text of the baseline book, `=== PAGE N ===` markers = PDF pages. The PDF-page start of every Choset chapter/appendix is tabulated in `TOC.md §4`; book-page → PDF-page offsets drift (19 → 14), so use that table, never an offset rule. |
| `CLAUDE.md` | This file: how to build the book. |

**Current state: web edition complete (2026-10-12).** All 23 chapters are implemented in `web/`
(MDX + widgets + `web/lib/<module>` + `__checks_chNN__.ts`); `npm run verify` passes strictly
(307 numerical checks, 24 pages render, 82 widgets, 174 references) and the site builds both at `/`
and under the GitHub Pages sub-path. The site chrome follows the Shannon Dynamics website reskin
(`app/shannon.css`, `components/shannon/*`); chapter figures keep the planning palette.
`pr-latex/` converts all 23 chapters (`make convert`); typesetting needs a TeX install (TinyTeX at
`~/.TinyTeX` or `pdflatex` on PATH) that this machine does not have yet.

The running lab: robots **Rusty** (disc / differential drive, shared with the sister book),
**Reach** (planar 2R arm; 3R variant), **Hitch** (car, optional single trailer); worlds **Apartment**
(shared), **Workbench**, **Lot**. Use these names and no others.

## Repository layout (target; mirrors the sister book)

```
TOC.md, Chapter-01.md … Chapter-23.md     # design docs (repo root) — the specs
web/                                        # forked from ../ProbabilisticRoboticRust/web
  app/  content/chapters/  components/{book,sim,viz,ch/chNN,home}/
  lib/
    prob/ geom/ sim/ mapping/edt            # PORTED from the sister book, unchanged
    robots/ collide/ cspace/ search/ potential/ roadmap/ cells/
    sampling/ multi/ estimate/ dynamics/ trajectory/ trajopt/ nonholonomic/ learn/ capstone/
    __checks__.ts  katex-macros.ts  book-structure.ts
  AUTHORING.md                              # forked; chapter skeleton and widget rules
pr-latex/                                   # forked print pipeline
Resource/                                   # baseline PDF + extracted text (ignored)
```

**Two implementations, on purpose.** The Rust in the prose is canonical and teachable. `web/lib/`
is a faithful TypeScript port that runs the in-page simulations. When you change one, change the
other, and make sure the chapter's worked numeric example still holds in both.

## Toolchain & commands (identical to the sister book)

- **Node ≥ 20.9**; this machine's Node 24 lives at `~/.local/node` — prefix with
  `export PATH="$HOME/.local/node/bin:$PATH"`.
- `cd web && npm run dev | build | typecheck | check | smoke | check:book | verify | preview`.
  `smoke` compiles and server-renders every chapter in seconds — run it before `build`: in MDX a
  bare `{…}` in prose is a JavaScript expression, and a `$…$` span broken across a line that starts
  with `+`/`-`/`*` becomes a list item, so its braces leak. `ALLOW_MISSING=1 npm run check:book`
  tolerates chapters not yet written; `npm run build` regenerates the sidebar from the files present.
- Rust versions for printed listings: the crate table in `TOC.md §2`.

## Non-negotiable conventions

1. **Notation** comes from `TOC.md §2` (Choset-compatible: $\mathcal{Q}$, $\mathcal{QO}_i$,
   $\mathcal{Q}_{free}$, $R(q)$, $c:[0,1]\to\mathcal{Q}_{free}$, …). Each chapter may only *add*
   symbols via its own notation table. Where a chapter meets the sister book's $x_t, u_t, z_t$,
   it says once that the planner's $q$ and the filter's $x_t$ coincide for Rusty.
2. **Color code** everywhere (figures, KaTeX terms, code comments, widget UI):
   start **green** · goal **red** · workspace obstacle **slate** · C-obstacle **amber (hatched)** ·
   roadmap/tree/explored **blue** · solution path **purple** · robot & executed trajectory **orange**.
   CSS custom properties `--rm-start`, `--rm-goal`, `--rm-obstacle`, `--rm-cobstacle`, `--rm-graph`,
   `--rm-path`, `--rm-robot`, defined once in `web/app/global.css`, redefined for dark mode. The
   sister book's `--pr-*` estimation roles are imported unchanged where a belief appears
   (Chs. 15–16, the capstone's belief panel). Never hardcode a hex anywhere else. Page chrome is the
   Shannon theme (`app/shannon.css`, accent `#4C7EFF`); in-figure chrome stays teal (`--pr-widget-ink`).
3. **Determinism:** every stochastic demo/test is seeded, with a visible seed control. Rust:
   `SmallRng`/`Pcg64`, never `thread_rng()`. Web: the ported `lib/prob/rng.ts`, never `Math.random()`.
4. **Widgets:** autoplay a sensible default; foreground one meaningful parameter; `teaches` names
   the misconception the widget kills; every widget wrapped in `WidgetFrame` with a caption; ids
   `w<chapter>.<k>` must match the design doc's manifest table.
5. **Hand-rolling policy:** planners, search, samplers, potentials, roadmaps, dynamics, time
   scaling, steering, and controllability tests are **written in the book**. Collision geometry
   (`parry2d`), exact segment Voronoi, and ML tensors (`candle`) are **delegated** and explained.
   `pathfinding`, `rapier2d`, `k`, `argmin` appear only as cross-validation in tests.
6. **Ownership of shared artifacts** (introduce once, reuse after) — the table in `TOC.md §2` is
   authoritative. In short: `Collision` + worlds + Reach FK → Ch. 2; range-sensor geometry +
   curve tracer → Ch. 3; Minkowski/star + Jacobian → Ch. 4; `Manifold` → Ch. 5; search → Ch. 6;
   brushfire → Ch. 7; Morse vocabulary → Ch. 8; sampler + kd-tree + `Steer` → Ch. 11; arm
   dynamics → Ch. 17; Dubins/RS + forward-propagation planner → Ch. 21. A later chapter must not
   re-introduce these; on the web side a widget imports from `web/lib/`, never reimplements.
7. **Every worked numeric example in the text is reproduced by a test** — a Rust unit test in the
   prose, and an invariant in `web/lib/__checks__.ts` where the port covers it. Property-test
   geometric claims against brute force (star algorithm vs. exhaustive Minkowski sum; Dubins words
   vs. forward integration; A\* vs. Dijkstra cost).
8. **Algorithm names** follow Choset's algorithm numbering where it exists (`BASIC PRM`,
   `TANGENT BUG`, `CAR GRID SEARCH`, …) alongside standard names (`a_star`, `rrt_star`, `chomp`).
9. Derivations: statement-first, 3–8 named steps in the text, full algebra in a collapsible
   block. Concrete before abstract; theorem-proof ordering only inside Foundation sections.
10. **Relationship to the sister book's planning code:** this book is the canonical, deeper
    treatment; the sister's Ch. 20 is a preview. Port only `sim`/`geom`/`prob`/`edt` (plus the
    sister's `filters`/`localize` vendored into Part IV's thin `estimate` crate); never copy its
    `lib/plan`. Cross-book links use the sister's published URL
    (`https://shannon-dynamics.github.io/ProbabilisticRoboticRust/chapters/<slug>`).

## Workflow

### Phase A (complete): design docs
Keep the 8-section skeleton used by all `Chapter-NN.md` files (Purpose/Story Arc; Prerequisites &
Position; Foundation; Conceptual; Practical; Manifest; Exercises; Modernization Notes). Never
renumber chapters or reuse widget ids; `TOC.md` and all cross-references move in the same change.
Every Choset section claimed in a doc's §2 must match the `TOC.md §4` source map. Modern citations
not yet verified carry "(verify)" until someone checks authors/year/venue/DOI.

### Phase B (complete): implementing a chapter
1. Fork `web/` and `pr-latex/` from the sister book; replace `lib/book-structure.ts`,
   `content/chapters/meta.json`, color tokens, and KaTeX macros per `TOC.md`; port
   `lib/{prob,geom,sim,mapping/edt}` unchanged; delete the sister's other `lib/` modules and
   `components/ch/*`.
2. Read `Chapter-NN.md` and the sister book's `web/AUTHORING.md`; `ch05-bayes-filter.mdx` and
   its widgets remain the quality bar for MDX and widget code.
3. Library: add the chapter's algorithms under the matching `web/lib/` subdirectory, with the
   chapter's invariants in `lib/<module>/__checks_chNN__.ts` exporting `runChNNChecks()`, then
   register that function in the `CHAPTER_CHECKS` list of `lib/__checks__.ts` and the module's
   exports in `lib/index.ts`.
4. Widgets: build the manifest entries in `web/components/ch/chNN/`, importing from `web/lib/`.
5. Prose: `web/content/chapters/chNN-slug.mdx` following the FCP rhythm, with a verified epigraph,
   Rust listings, a hand-checkable worked example, and verified citations.
6. Gates: `npm run verify` clean; widget ids and colors matching the manifest; notation matching
   `TOC.md`; every "(verify)" citation resolved or removed.

**Definition of done** = step 6 plus: cross-chapter and cross-book links resolve, exercises span
F/C/P, every citation and the epigraph verified to exist, and the design doc's §8 modernization
claims actually reflected in the text.

## Style

- Voice: rigorous but narrative-first; honest about approximations and failure modes; playful is
  fine, imprecise is not. American spelling.
- Rust: idiomatic, `clippy`-clean, const generics for dimensions where natural, newtypes for
  configuration spaces; a deliberate compile-error example (mixing $T^2$ and $SE(2)$
  configurations) is a feature of Practical sections, not a bug.
- No placeholder content in the published book — every widget listed in a manifest exists or the
  chapter doesn't ship.
