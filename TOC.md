# Principles of Robot Motion via Rust

**An interactive web book — Foundation, Conceptual, Practical (FCP)**

> Every planner derived rigorously, every configuration space made visible and manipulable in the
> browser, every algorithm implemented in production-quality Rust — and the *same* Rust code that
> appears in the text powers the interactive simulations on the page.

Sister volume of *Probabilistic Robotics via Rust* (`../ProbabilisticRoboticRust/`). Same method,
same publishing architecture, same robot Rusty — a different question: not *where am I?* but
*how do I get there?*

---

## 1. Vision & Positioning

Choset, Lynch, Hutchinson, Kantor, Burgard, Kavraki, and Thrun's *Principles of Robot Motion*
(MIT Press, 2005) is the textbook that made motion planning a discipline with a shared vocabulary:
configuration space, roadmaps, cell decompositions, sampling-based planning, and the differential
geometry behind nonholonomic systems, all in one volume. Twenty years on, the field has moved:
sampling-based planners are now *asymptotically optimal* (RRT\*, BIT\*, FMT\*), trajectory
optimization (CHOMP, TrajOpt, iLQR/MPC) sits beside search as the production workhorse, planning
libraries (OMPL, MoveIt, Nav2) made the algorithms infrastructure, and learning has entered the
loop. No book teaches this material with in-page simulations, and none teaches it in Rust — the
language the robotics industry is adopting for safety-critical autonomy.

This book fills that gap with three promises:

1. **Foundation (F)** — every concept gets its full mathematical formalism: definitions, stated
   assumptions, derivations and proof sketches done honestly (collapsible for skimmers, complete for
   graduate readers), in Choset's notation extended where the modern literature demands it.
2. **Conceptual (C)** — every hard idea gets a visual, interactive treatment. The reader drags a
   workspace obstacle and watches a C-obstacle deform on a torus; grows an RRT and sees the Voronoi
   bias; scrubs a phase-plane trajectory to the velocity-limit curve; parallel-parks a trailer with
   Lie brackets. Interaction is invitation, not requirement — every widget autoplays a sensible
   default and degrades to a static figure without JS.
3. **Practical (P)** — every algorithm is implemented in idiomatic Rust with the best current crates,
   inside one cargo workspace where the printed listings, the test suite, the figures, and the
   in-page demos are literally the same code.

**What this book is not.** It is not a second estimation book. Choset's Chapters 8–9 (Kalman and
Bayesian methods) are kept — compressed to what a *planner* needs and told through Choset's
distinctive geometric-observer lens — and every deep question is cross-linked to the sister
volume, which treats them in twenty-six chapters.

### The running lab

Three robots and three worlds thread through the whole book:

| Robot | What it is | Home world | Carries |
|---|---|---|---|
| **Rusty** (shared with the sister book) | disc / differential-drive rover | **The Apartment** — the sister book's 2D floorplan, polyline walls | point & disc planning, Parts I–IV, the capstone's navigation panel |
| **Reach** | planar 2R arm (3R variant for redundancy and underactuation) | **The Workbench** — a table with polygonal obstacles | configuration-space topology (its C-space is a torus), potentials for articulated bodies, dynamics and trajectory planning |
| **Hitch** | car-like robot, optionally pulling one trailer | **The Lot** — a parking lot with bays and pillars | nonholonomic constraints, controllability, steering, parking |

Rusty is built once in Chapter 2 (ported from the sister book's simulator, never re-derived) and
Reach and Hitch are built beside it. By Chapter 23 all three act at once: Rusty navigates an
Apartment it has never seen, Reach executes a time-optimal collision-free trajectory, Hitch parks
the trailer.

### Reader path per chapter (fixed rhythm)

Hook → **C**onceptual (play first) → **F**oundation (now the rigor) → **P**ractical (Rust) →
Integration lab (run it in the running worlds) → Exercises. Concrete before abstract, always;
theorem-proof ordering is used only *inside* Foundation sections.

---

## 2. Book-wide Conventions

### Notation (Choset-compatible; Choset's Appendix A is folded into this table)

| Symbol | Meaning |
|---|---|
| $\mathcal{W} \subset \mathbb{R}^2$ or $\mathbb{R}^3$, $\mathcal{WO}_i$, $\mathcal{W}_{free}$ | workspace, $i$-th workspace obstacle, free workspace |
| $\mathcal{Q}$, $q$, $\dim \mathcal{Q}$ | configuration space, a configuration, degrees of freedom |
| $R(q) \subset \mathcal{W}$ | the set of workspace points occupied by the robot at $q$ |
| $\mathcal{QO}_i = \{q : R(q) \cap \mathcal{WO}_i \ne \emptyset\}$, $\mathcal{Q}_{free}$ | C-obstacle, free configuration space |
| $c : [0,1] \to \mathcal{Q}_{free}$, $c(t)$ | a path (geometric curve); a trajectory when parameterized by time |
| $q_{start}$, $q_{goal}$ | query endpoints |
| $d(x,y)$, $d_i(q)$, $D(x)$ | metric; distance from $q$ to C-obstacle $i$; distance from workspace point $x$ to the nearest obstacle |
| $\rho(x, s)$, $\rho_R(x,s)$ | raw distance along ray at angle $s$ from $x$; saturated (range-limited) version |
| $U(q) = U_{att}(q) + U_{rep}(q)$, $\nabla U$ | artificial potential and its gradient |
| $\varphi : \mathcal{Q} \to \mathbb{R}^n$, $D\varphi$, $J(q)$ | forward kinematic map, its differential, the Jacobian |
| $S^1$, $T^n$, $SO(n)$, $SE(n)$, $\mathbb{RP}^n$ | circle, torus, rotation and rigid-motion groups, projective space |
| $T_q\mathcal{Q}$, $f(q)$, $g_i(q)$ | tangent space; drift and control vector fields |
| $[f, g]$, $\Delta$, $\overline{\Delta}$ | Lie bracket; distribution; its involutive closure |
| $M(q)$, $C(q,\dot q)$, $g(q)$, $\tau$ | inertia matrix, Coriolis/centrifugal matrix, gravity vector, joint torques |
| $\omega(q)\,\dot q = 0$ | Pfaffian (velocity) constraint |
| $G = (V, E)$, $g(v)$, $h(v)$, $f(v)$ | graph; cost-to-come, heuristic, A\* priority |
| $n$, $k$, $r_n$ | sample count, neighbor count, connection radius (sampling-based planners) |
| $(\epsilon, \alpha, \beta)$ | expansiveness parameters of $\mathcal{Q}_{free}$ |
| $x_t$, $u_t$, $z_t$, $bel(x_t)$ | **imported from the sister book** for Part IV only; the text says once, precisely, that the planner's $q$ and the filter's $x_t$ coincide for Rusty |

A per-chapter notation table lists exactly the symbols that chapter introduces. KaTeX with one
global macro file keeps rendering consistent; the macro table extends the sister book's with
`\Qfree`, `\QO`, `\WO`, `\Rq`, `\lie{}{}`, `\dist`.

### The book color code (figures, equations, and code comments all agree)

Planning has different roles than estimation, so this book defines its own code. The sister
book's five estimation colors (`--pr-prior/-prediction/-measurement/-posterior/-truth`) are
imported *unchanged* wherever a belief appears (Chapters 15–16, the capstone's belief panel).

| Role | Color | CSS variable |
|---|---|---|
| Start configuration | **green** | `--rm-start` |
| Goal configuration | **red** | `--rm-goal` |
| Workspace obstacle | **slate** (solid fill) | `--rm-obstacle` |
| C-obstacle | **amber** (hatched) | `--rm-cobstacle` |
| Roadmap / tree / explored set | **blue** | `--rm-graph` |
| Solution path | **purple** | `--rm-path` |
| Robot & executed trajectory | **orange** | `--rm-robot` |

Exact hues are fixed at implementation time with light/dark contrast checks. Page chrome uses the
Shannon Dynamics accent (`#4C7EFF`); in-figure chrome (scrubbers, role dots, canvas highlights)
stays teal (`--pr-widget-ink`), deliberately outside the data hues. **Purple clash rule:** where a belief (`--pr-posterior`,
purple) and a solution path (`--rm-path`, purple) share a canvas (Ch. 16, the capstone), the belief
is drawn as filled cells/dots and the path as a thick haloed stroke; the pair must pass a contrast
check before the widget ships. In equations, tint terms with `\htmlClass{term-start}{…}`
and friends; in Rust, the `Role` enum in the widget kit is the only place a color is named.

### Shared-artifact ownership (introduce once, import after)

A later chapter must not re-introduce these. On the web side the same rule applies to `web/lib/`.

| Artifact | Introduced in | Imported by |
|---|---|---|
| `Collision` trait + parry2d black box; the three worlds; Reach's **forward kinematics** (Choset §3.8 FK) | Ch. 2 | everyone; Ch. 4 opens the collision box |
| Range-sensor geometry $D(x)$, $\rho(x,s)$, $\nabla D$; the **curve tracer** `trace_curve` (Choset App. D predictor–corrector) | Ch. 3 | Ch. 7 §4.3.1, Ch. 8 §5.2.5, Ch. 9 §5.3.3, Ch. 10 §6.2.4 |
| Polygon Minkowski sums / star algorithm, $SE(2)$ C-obstacle slices; Reach's **Jacobian & singularities** (§3.8) | Ch. 4 | Ch. 7 §4.7.3, Chs. 17–18, 21 |
| `trait Manifold` with $S^1$, $T^2$, $SE(2)$, $SO(3)$ impls; metrics (App. C.2) | Ch. 5 | Ch. 11 (kd-tree metrics), Ch. 20 |
| Graph search: BFS / Dijkstra / A\* / D\* / D\* Lite | Ch. 6 | Ch. 7 (wave-front and brushfire *are* BFS), Chs. 8, 10, 18, 21 |
| Grid distance transform (brushfire) | Ch. 7 | Ch. 8 §5.2.5 grid GVD, Ch. 10 §6.2.4; cross-links the sister book's ESDF (its Ch. 19) |
| Morse-function / critical-point vocabulary (definition box beside the preimage theorem) | Ch. 8 | Ch. 9 (Canny), Ch. 10 (Morse decompositions) |
| C-space sampler (uniform, Gaussian, bridge, OBPRM, medial-axis); `trait Chart: Manifold` (coordinates + periodic axes) and the kd-tree NN over it; `trait Steer` (straight-line local planner) | Ch. 11 | Ch. 12 adds `extend`; Ch. 13; Ch. 21 adds Dubins / Reeds–Shepp / flat steering |
| Arm dynamics $M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \tau$ | Ch. 17 | Chs. 18–21 |
| Hand-rolled Gauss–Newton / Levenberg–Marquardt optimizer and iLQR (`trajopt::optim`) | Ch. 19 | Ch. 21 (nonlinear steering), Ch. 23 |
| Dubins / Reeds–Shepp closed forms; forward-propagation (kinodynamic) tree planner | Ch. 21 (a one-page *preview* of forward propagation lives in Ch. 14) | Ch. 23 |

### Rust crate stack (candidates — each verified for WASM before adoption, as the sister book did)

| Layer | Primary | Notes / alternates |
|---|---|---|
| Linear algebra | `nalgebra` 0.35 | `SMatrix`/`SVector` for configurations, Jacobians, inertia matrices; WASM-clean |
| Collision & distance | `parry2d` 0.30 (`parry2d-f64`), `parry3d` for the one 3D showpiece | GJK/EPA, BVH, shape-cast for swept edges. **The one thing this book does not hand-roll** — Ch. 4 explains what it does, App. C documents it; parry ≥ 0.26 does its math in glam (`glamx`), so `collide` converts poses and points to and from nalgebra at its boundary, and no other crate sees a parry type |
| Dynamics cross-check | `rapier2d` | tests only: our hand-rolled Lagrangian for Reach must match the engine |
| Graphs & search | `petgraph` 0.8, `pathfinding` 4.15 | roadmaps and lattices; `pathfinding` cross-checks our hand-rolled A\*/Dijkstra |
| Randomness | `rand` 0.9 (`SmallRng`), `rand_pcg` 0.9 (`Pcg64`), `rand_distr` 0.5 | seeded everywhere; never `thread_rng()`. `Pcg64` lives in `rand_pcg`, not `rand::rngs`; `rand_distr` 0.5 is the release built on `rand` 0.9 |
| Nearest neighbors | `kiddo` (or hand-rolled kd-tree in Ch. 11) | needed in $T^2$, $SE(2)$, $\mathbb{R}^d$ with custom metrics — verify the crate's metric hooks; hand-roll if they are missing |
| Arm kinematics cross-check | `k`, `urdf-rs` | Reach's FK/Jacobian validated against a URDF model |
| Optimization | hand-rolled GN/LM + iLQR; `argmin` as the comparison library | Ch. 19 trajectory optimization, Ch. 21 nonlinear steering |
| Exact Voronoi of segments | `boostvoronoi` vs. grid brushfire | Ch. 8 GVD: exact vs. approximate, both shown |
| ML (Ch. 22) | `candle` (or `burn`) | learned samplers and heuristics, small nets only |

Hand-rolling policy: planners, search, samplers, potentials, roadmaps, dynamics, time scaling,
steering, and controllability tests are **written in the book**; collision geometry, exact
Voronoi, and ML tensors are **delegated** to crates and explained.

### Relationship to the sister book's code

The sister book's Chapter 20 ("Motion Planning: From Geometry to Probability") specifies
`CSpace2`, `trait Steer`, `RrtStar`, `Dubins`, and a hybrid-A\* sketch in its `ch20_planning`
crate and `web/lib/plan/`. **This book is the canonical, deeper treatment; the sister's Chapter 20
is a preview and cross-links here.** What is shared:

- **Ported, not re-derived:** `sim` (Rusty, the Apartment, ray-cast range sensor), `geom` (SE(2)),
  `prob` (seeded RNG), `mapping/edt` (exact distance transform). Chapter 2 imports them. Part IV
  additionally vendors the sister's `filters/` (KF/EKF/grid/particle) and `localize/` ports as a
  thin `estimate` crate — wrapped, never redesigned; Chs. 15–16 are the only importers.
- **This book's own, never copied from the sister's `lib/plan`:** every planning crate. Where the
  two books overlap (A\*, PRM, RRT, RRT\*, Dubins) the implementations here are the reference ones
  and the sister book's are the simplified versions; the text may say so.
- **Cross-book links** use the sister's published URL
  (`https://prvr.shannon.id/chapters/<slug>`) and are listed
  per chapter in §2 of each design doc.

### Publishing architecture (web) — inherited

Identical to the sister book, by decision: the `web/` scaffolding (Next.js 16 App Router static
export, React 19, Fumadocs 16, MDX per chapter, `remark-math` + `rehype-katex`, Shiki, Nivo 0.99,
Canvas 2D sim framework with `useSimulation`/`SimCanvas`/`WidgetFrame`, Tailwind v4, self-hosted
fonts), its `check`/`check:book`/`verify` scripts, the deploy setup, and the
`pr-latex` MDX→LaTeX print pipeline are forked from `../ProbabilisticRoboticRust/web` and
`pr-latex` when implementation begins. Only the book structure, the color tokens, and the
algorithm library change.

**The algorithms are implemented twice, on purpose.** The Rust in the text is the canonical,
teachable implementation. `web/lib/` holds a faithful TypeScript port that powers the in-page
simulations, pinned by numerical invariants in `lib/__checks__.ts` (e.g. Dubins words integrate to
their endpoints; the star algorithm equals the brute-force Minkowski sum on random polygons; A\*
with an admissible heuristic equals Dijkstra's cost; the time-optimal phase-plane trajectory
touches the velocity-limit curve exactly at switching points). Where the book states a worked
numeric example, both implementations must reproduce it.

---

## 3. Table of Contents

Legend per chapter: **[F]** foundation depth · **[C]** flagship widgets · **[P]** what gets built ·
**Sources** baseline mapping ("C" = Choset et al. 2005; "sister" = *Probabilistic Robotics via
Rust*). Full per-chapter designs live in `Chapter-01.md` … `Chapter-23.md`.

---

### Part I — Foundations: Robots, Worlds, and Configuration Space

**1. Robots That Plan: The Piano Mover's Problem**
Why motion planning is hard and worth it: the piano mover, the Mini AERCam, the arm, the molecule;
Choset's three-axis taxonomy (task: navigate/cover/localize/map · robot: DOF, omnidirectional vs
nonholonomic, kinematic vs dynamic · algorithm: optimality, complexity, completeness kinds,
online/offline, sensor-based); path vs. trajectory; the FCP method and the cast.
*[C]* **w1.1 Bug Preview** (Bug1 crawling around two obstacles — the thesis in one loop);
**w1.2 Piano Mover** (drag a sofa through a doorway while its C-space point moves beside it).
*[P]* Zero-dependency `ch01_hello`: Bug1 on a text-mode world; reader setup; first `cargo run`.
*Sources: C Ch. 1 (§1.1–1.3); pedagogy of the sister book's Ch. 1.*

**2. The Lab: Three Robots, Three Worlds, One Simulator**
Rusty (ported), Reach, Hitch; the Apartment (ported), the Workbench, the Lot; configurations as
typed values ($\mathbb{R}^2$, $T^2$, $SE(2)$, $SE(2)\times S^1$); Reach's forward kinematics;
collision and distance queries through `parry2d` as a black box (opened in Ch. 4); the range sensor
Bug algorithms need; the widget framework; determinism as a feature.
*[C]* **w2.1 Three-Robot Dashboard** (drive Rusty, drag Reach's joints, steer Hitch — collision
flashes); **w2.2 Query Anatomy** (what a collision checker returns: boolean, distance, witness
points, swept volume).
*[P]* `sim` (ported) + `robots` (Reach FK, Hitch kinematics) + `collide` (the `Collision` trait over
parry2d) + `widget-kit`; the workspace layout fixed for the whole book.
*Sources: NEW (mirrors sister Ch. 4); C §3.8 (forward kinematics only); C App. F.5 as the black box.*

**3. Bug Algorithms: Planning with a Contact Sensor**
Sensor-based planning with provable guarantees from almost no mathematics: Bug1 (exhaustive,
safe), Bug2 (greedy, sometimes worse), Tangent Bug (a range sensor buys shorter paths); the
tangent line and continuity intervals; distance and gradient from range data; boundary following
as curve tracing via the implicit function theorem — the predictor–corrector the roadmap chapters
reuse.
*[C]* **w3.1 Bug Arena** (Bug1/Bug2/Tangent Bug race on the same map; find the map where Bug2
loses); **w3.2 Boundary Follower** (the curve tracer stepping along an obstacle at fixed offset,
Newton corrections visible).
*[P]* `bugs` crate: three planners over the Ch. 2 range sensor; `trace_curve(G, DG)` — an owned
artifact; property test: Bug1 path length $\le d(q_s,q_g) + 1.5\sum p_i$.
*Sources: C Ch. 2 (§2.1–2.3) + App. D; sonar-cone detail cross-linked to sister Ch. 4/10.*

**4. Configuration Space I: Obstacles in the Space of Configurations**
Specifying a configuration; the C-obstacle of a disc (Minkowski inflation) and of a two-joint arm
(the torus with holes); dimension and degrees of freedom; example C-spaces; the Jacobian of Reach
and its singularities ($\sin\theta_2 = 0$); opening the collision box: polygons, intersection
tests, the star algorithm for Minkowski sums, $SE(2)$ slices, GJK distances.
*[C]* **w4.1 C-Space Morph** (drag a workspace obstacle; Reach's C-obstacle deforms live on the
unwrapped torus — the book's signature figure); **w4.2 Minkowski Studio** (two polygons → star
algorithm → the C-obstacle, vertex by vertex); **w4.3 Jacobian Lens** (joint velocities → tip
velocity ellipse collapsing at singularities).
*[P]* `cspace` crate: C-obstacle rasterizer (any robot with a `Collision` impl), star algorithm with
brute-force cross-check, `jacobian()` for Reach validated against `k`.
*Sources: C §3.1–3.3, §3.7, §3.8 (Jacobian); App. F (F.1–F.5).*

**5. Configuration Space II: Topology, Manifolds, and Rigid Bodies**
Homeomorphisms and diffeomorphisms; differentiable manifolds and charts; connectedness and
compactness; C-spaces that are not manifolds; embeddings in $\mathbb{R}^n$ and matrix
representations of rigid-body configuration ($SO(2)$, $SE(2)$, $SO(3)$, $SE(3)$); metrics on
C-space; the differential $D\varphi$; the modern exponential-coordinates view, cross-linked.
*[C]* **w5.1 Torus Unwrapper** (the square with identified edges folds into the torus and back;
a path crosses the seam); **w5.2 Rotation Zoo** (Euler-angle gimbal lock vs. quaternion
continuity, same motion).
*[P]* `trait Manifold { fn dim; fn dist; fn interpolate; fn sample }` with $S^1$, $T^2$, $SE(2)$,
$SO(3)$ impls — the type every planner is generic over; compile-error demo for mixing spaces.
*Sources: C §3.4–3.6; App. B, C (C.1–C.5); App. E demoted to Appendix B card; sister Ch. 3 / App. C
for Lie groups.*

---

### Part II — Classical Planners: Search, Potentials, Roadmaps, Cells

**6. Graph Search for Planners**
Graphs, adjacency, grids as graphs; BFS and DFS; Dijkstra; greedy best-first; A\* with
admissibility and consistency proofs; the non-optimistic counterexample; D\* for replanning on a
changing grid; universal plans (backward Dijkstra — a first value function, cross-linked to the
sister book's MDP chapter); running time, complexity classes, and what "complete" means.
*[C]* **w6.1 Search Theater** (BFS / Dijkstra / greedy / A\* expanding on the same grid, heuristic
weight slider — watch optimality break); **w6.2 D\* Replanner** (a gate closes mid-run; only the
affected costs repair).
*[P]* `search` crate: generic over `petgraph` and grids; A\*/Dijkstra cross-checked against
`pathfinding`; D\* and D\* Lite; benchmark table reproduced by a test.
*Sources: C App. H (H.1–H.4), App. G (G.1–G.3); D\* Lite and ARA\* from the modernization set.*

**7. Potential Functions**
Attractive and repulsive potentials; gradient descent; computing distance in the plane (sensor
and grid); brushfire; the local-minimum disease; the wave-front planner; navigation functions on
sphere worlds and star worlds (Rimon–Koditschek); potentials in non-Euclidean spaces: lifting
workspace forces through $J^\top$, control points on rigid bodies and articulated arms.
*[C]* **w7.1 Potential Well** (drag the goal and a U-shaped obstacle; the bead gets stuck;
wave-front mode frees it); **w7.2 Navigation Function Sculptor** (sphere world → star world
diffeomorphism, $\kappa$ slider, the single minimum surviving); **w7.3 Arm in a Field** (Reach's
control points pulled by workspace forces, torques shown in joint space).
*[P]* `potential` crate: potentials over the Ch. 4 distance queries; `brushfire`/`wavefront` on the
grid (owned distance transform); navigation function for sphere/star worlds.
*Sources: C Ch. 4 (§4.1–4.7).*

**8. Roadmaps I: Visibility Graphs and the Generalized Voronoi Diagram**
Roadmap properties (accessibility, connectivity, departability); the visibility graph and its
rotational-sweep construction; the GVD as a deformation retract; the preimage theorem, regular
and critical values, Morse functions (defined here, reused in Chs. 9–10); GVD dimension and
construction (exact for segments, approximate by brushfire ridges).
*[C]* **w8.1 Rotational Sweep** (the sweep line and the visible-vertex list updating edge by
edge); **w8.2 Voronoi Carver** (brushfire fronts meet → ridges; overlay the exact GVD).
*[P]* `roadmap` crate: visibility graph (reduced and full), GVD via `boostvoronoi` and via the Ch. 7
brushfire; retraction of a query onto the roadmap; A\* on the roadmap.
*Sources: C §5.1 (5.1.1–5.1.2), §5.2 (5.2.1–5.2.5).*

**9. Roadmaps II: Sensor-Based Roadmaps and Silhouettes**
The generalized Voronoi graph in higher dimensions; transversality; connectivity and the
hierarchical GVG; sensor-based construction by Lyapunov control (Rusty traces the GVG from range
data — the curve tracer again); the rod-HGVG; silhouette methods: Canny's roadmap and the
opportunistic path planner; the complexity of exact motion planning.
*[C]* **w9.1 GVG Explorer** (Rusty, range sensor only, incrementally traces the Apartment's GVG;
meet points and boundary edges accumulate); **w9.2 Silhouette Sweep** (a slicing plane sweeps a
3D C-space; silhouette curves and linking curves appear).
*[P]* `roadmap::gvg` (incremental, sensor-based) + a small Canny demonstrator on a synthetic 3D
C-space; benchmark: GVG path vs. shortest path.
*Sources: C §5.3 (5.3.1–5.3.3), §5.4, §5.5 (5.5.1–5.5.2).*

**10. Cell Decompositions and Coverage**
Trapezoidal decomposition and its adjacency graph; Morse decompositions (boustrophedon, variable
slice); sensor-based coverage with critical-point detection; the complexity of coverage;
visibility-based decompositions for pursuit/evasion (coda); modern coverage planning pointers.
*[C]* **w10.1 Boustrophedon Mower** (the slice sweeps, cells split and merge at critical points,
the lawnmower path covers every cell); **w10.2 Pursuit-Evasion** (information states as the
pursuer moves; the gap edges that matter).
*[P]* `cells` crate: trapezoidal decomposition (sweep), boustrophedon coverage planner over the
Workbench and the Apartment; coverage completeness test.
*Sources: C Ch. 6 (§6.1–6.3).*

---

### Part III — Sampling-Based Planning

**11. Probabilistic Roadmaps**
Trading completeness for speed; basic PRM (learning and query phases); a practical
implementation: metrics on $T^2$ and $SE(2)$, kd-trees, the local planner, collision checking
schedules; sampling strategies (Gaussian, bridge, OBPRM, medial-axis) against narrow passages;
connection strategies and lazy evaluation (Lazy PRM, classical); analysis: PRM in $\mathbb{R}^d$,
$(\epsilon,\alpha,\beta)$-expansiveness, abstract path tiling, probabilistic completeness stated
precisely.
*[C]* **w11.1 Roadmap Rain** (samples fall, edges connect, components merge; the moment the query
connects); **w11.2 Narrow Passage** (uniform vs. Gaussian vs. bridge vs. OBPRM, success rate vs.
corridor width measured live).
*[P]* `sampling` crate: `Sampler` strategies, kd-tree NN, `trait Steer`, `Prm` generic over
`Manifold`; the owned artifacts Chs. 12–13 and 21 import.
*Sources: C §7.1 (7.1.1–7.1.4), §7.4 (7.4.1–7.4.3).*

**12. Tree Planners: EST, RRT, and Friends**
Single-query planning; expansive-spaces trees; rapidly-exploring random trees and the Voronoi
bias; RRT-Connect and SBL (bidirectional, lazy collision checking); the sampling-based roadmap of
trees (SRT) that degenerates to PRM, RRT, or EST; KPIECE and projection-based exploration.
*[C]* **w12.1 Tree Grower** (RRT growing in the Workbench's torus C-space; toggle the Voronoi
regions to see why it explores); **w12.2 Bidirectional Connect** (two trees reaching for each
other; greedy connect steps).
*[P]* `sampling::tree`: `Est`, `Rrt`, `RrtConnect`, `Sbl`, `Srt` sharing one tree type and the
Ch. 11 `Steer`; benchmark on three worlds.
*Sources: C §7.2 (7.2.1–7.2.3), §7.3; KPIECE from the modernization set.*

**13. Optimal Sampling-Based Planning**
Why RRT's path never improves (almost surely suboptimal); RRT\* and PRM\*: choose-parent and
rewire, the $r_n = \gamma (\log n / n)^{1/d}$ radius, asymptotic optimality stated precisely
(Karaman–Frazzoli, with the radius-constant correction); Informed RRT\* and the prolate
hyperspheroid; BIT\* and FMT\* as batch methods; what "anytime" buys in practice.
*[C]* **w13.1 Rewire Watch** (RRT vs. RRT\* on the same samples; the cost curve descending;
rewired edges flash); **w13.2 Ellipse Shrinker** (the informed set tightening around the
incumbent path).
*[P]* `sampling::optimal`: `RrtStar`, `PrmStar`, `InformedRrtStar`, `Bit`, `Fmt`; cost-convergence
test under a fixed seed.
*Sources: NEW — Karaman & Frazzoli 2011; Gammell et al. 2014/2015; Janson et al. 2015; Solovey et
al. 2020 (all to be verified at authoring time).*

**14. Planning for Many Bodies: Multi-Robot, Manipulation, and Kinodynamic Previews**
Composite configuration spaces and coupled vs. decoupled multi-robot planning; conflict-based
search; manipulation planning as modes (transit/transfer) and the task-and-motion planning view;
a one-page preview of forward-propagation planning ("plan with a simulator instead of a steer" —
Hitch's first planner, full treatment in Ch. 21); OMPL and MoveIt as systems; assembly, flexible
objects, and biology as pointers.
*[C]* **w14.1 Two-Robot Composite C-space** (two discs in a corridor; the 4D C-space shown as
coupled slices; prioritized vs. coupled plans); **w14.2 Pick-and-Place Modes** (Reach moves a block
through transit/transfer manifolds); **w14.3 Hitch Without a Steer** (forward-propagated RRT for
the car, no steering function).
*[P]* `multi` crate: composite `Manifold`, prioritized planning, CBS on grids; forward-propagation
RRT over the Ch. 2 Hitch kinematics.
*Sources: C §7.5 (7.5.1 preview, 7.5.2–7.5.3 full, 7.5.4–7.5.6 pointers); CBS, TAMP survey, OMPL
from the modernization set.*

---

### Part IV — Planning with Uncertainty (compressed; cross-linked to the sister book)

**15. Kalman Filtering: The Geometric Observer**
Probabilistic estimation as a planner's problem; a simple deterministic observer (Luenberger) and
what its gain means; observing with probability distributions; the Kalman filter as the
precision-optimal observer; the dead-reckoning example; observability of linear systems (rank
test); the extended Kalman filter for range-and-bearing and range-only localization; data
association; Kalman SLAM. Depth questions cross-linked to the sister book.
*[C]* **w15.1 Observer Bench** (Luenberger → Kalman as a gain schedule on Rusty's dead reckoning);
**w15.2 Observability Probe** (one landmark vs. two; the rank test and the ellipse that never
shrinks).
*[P]* `estimate` crate (thin): observer, KF, EKF range-bearing localizer over the Ch. 2 sim; SLAM
demo; cross-validated against the sister book's `filters/`.
*Sources: C Ch. 8 (§8.1–8.4), App. J.1, J.4, J.5; App. I replaced by sister Ch. 2; sister Chs. 5–7,
11, 14.*

**16. Bayesian Localization and Mapping**
Probabilistic localization as recursive Bayesian filtering, derived; representations of the
posterior (Kalman, grid, particles); sensor models; mapping with known poses; Bayesian SLAM; and
the planner's view: *replanning on a belief* — closing the loop that Parts II–III left open.
*[C]* **w16.1 Grid vs Particles** (same log, two posteriors, cost measured); **w16.2
Plan-on-Belief** (Rusty replans as the particle cloud condenses; the path hugs information).
*[P]* `estimate::bayes` (thin): grid and particle localizers wrapping the sister book's port;
`replan_on_belief` loop combining Ch. 6 search with a sampled belief.
*Sources: C Ch. 9 (§9.1–9.2); sister Chs. 8, 10, 12, 13, 17.*

---

### Part V — Dynamics, Trajectories, and Constraints

**17. Robot Dynamics**
Lagrangian dynamics from kinetic and potential energy; the standard form $M\ddot q + C\dot q + g =
\tau$ and Christoffel symbols; velocity (Pfaffian) constraints and Lagrange multipliers; rigid-body
dynamics: planar rotation, spatial rotation and the inertia tensor.
*[C]* **w17.1 Lagrangian Lab** (Reach swinging under gravity; energy bars; toggle Coriolis terms
off and watch the physics lie); **w17.2 Inertia Ellipsoid** (the configuration-dependent $M(q)$
as an ellipse in joint-velocity space).
*[P]* `dynamics` crate: symbolic-by-hand Lagrangian for Reach (2R/3R), `rapier2d` cross-check;
RK4 integrator with energy-drift test.
*Sources: C Ch. 10 (§10.1–10.4).*

**18. Trajectory Planning**
From path to trajectory; decoupled planning: path-velocity decomposition, the phase plane
$(s, \dot s)$, the velocity-limit curve, bang-bang switching, zero-inertia points, global
time-optimal trajectory planning; grid-based search over $(q, \dot q)$ — the state-lattice
ancestor; modern time-optimal path parameterization (TOPP-RA).
*[C]* **w18.1 Phase-Plane Racer** (drag the path; the velocity-limit curve and the
maximum-acceleration/deceleration curves redraw; switching points found live); **w18.2 Lattice
Hopper** (search over $(q,\dot q)$ cells with motion primitives).
*[P]* `trajectory` crate: `time_scale(path, dynamics, limits)` (Bobrow / Shin–McKay), TOPP-RA
variant, $(q,\dot q)$ grid search on Ch. 6 A\*.
*Sources: C §11.1, §11.2 (11.2.1–11.2.2), §11.3.3; TOPP-RA from the modernization set.*

**19. Trajectory Optimization and MPC**
Direct trajectory planning as history (optimal control, nonlinear optimization); the modern
family: CHOMP (functional gradient on a distance field), STOMP (stochastic), TrajOpt (sequential
convex with collision constraints), iLQR/DDP, model predictive control; where optimization beats
search and where it gets stuck. Cross-linked to the sister book's MPPI chapter.
*[C]* **w19.1 CHOMP Smoother** (a jagged RRT path for Reach relaxes under obstacle and smoothness
gradients; step size slider); **w19.2 iLQR Backward Pass** (value-function quadratics propagate
backward along Hitch's trajectory; forward pass follows).
*[P]* `trajopt` crate: CHOMP over the Ch. 7 distance field, iLQR over the Ch. 17 dynamics, MPC loop
on Rusty; `argmin` as comparison.
*Sources: C §11.3.1–11.3.2 (as history), §12.5.3 cross-linked; CHOMP, STOMP, TrajOpt, iLQR/DDP from
the modernization set; sister Ch. 23.*

**20. Nonholonomic Systems I: Controllability**
Tangent spaces and vector fields; distributions and Pfaffian constraints; the Lie bracket as the
motion you get by wiggling; control systems with drift; local accessibility and controllability
(Chow's theorem); simplified controllability tests for simple mechanical control systems;
mechanical systems with nonholonomic constraints.
*[C]* **w20.1 Lie Bracket Parallel-Parker** (steer/drive/−steer/−drive on Hitch; the net sideways
displacement is the bracket); **w20.2 Reachable Set Grower** (accessible set from a configuration
under one, two, then bracketed inputs).
*[P]* `nonholonomic` crate: `VectorField`, `lie_bracket`, filtration/rank tests for the unicycle,
car, and car-with-trailer; property test: bracket antisymmetry and Jacobi identity.
*Sources: C §12.1 (12.1.1–12.1.3), §12.2, §12.3 (12.3.1–12.3.2), §12.4.1, §12.4.3.*

**21. Nonholonomic Systems II: Steering Cars, Trailers, and Underactuated Systems**
Optimal control for steering (Dubins and Reeds–Shepp as closed forms); chained-form systems and
sinusoidal steering; nonlinear optimization and gradient methods for driftless systems;
differential flatness for the car and trailers; kinematic reductions for planning; cars and
cars pulling trailers; forward-propagation (kinodynamic) planning in full; state lattices and
hybrid A\* as descendants of Choset's CAR GRID SEARCH; Kinodynamic RRT\*.
*[C]* **w21.1 Trailer Parker** (drag Hitch and the trailer into a bay; flat-output path, sinusoids,
or RS primitives — each shown); **w21.2 Flatness Follower** (draw a flat-output curve; the full
state follows exactly); **w21.3 Lattice vs Hybrid A\*** (same lot, two planners, primitive sets
visible).
*[P]* `nonholonomic::steer`: Dubins/RS, sinusoidal steering for chained forms, flatness-based
steering; `kinodynamic` RRT and RRT\*; hybrid A\*.
*Sources: C §12.4.2, §12.5 (12.5.1–12.5.8), §7.5.1 (full); hybrid A\*, state lattices, Kinodynamic
RRT\* from the modernization set.*

---

### Part VI — Frontiers and Integration

**22. Learning and Planning**
Learning inside the planning loop, not instead of it: learned sampling distributions, learned
heuristics and cost-to-go, neural motion planners, diffusion-based planners; what transfers, what
overfits, and how to benchmark honestly against the classical baselines of Parts II–III.
*[C]* **w22.1 Learned Sampler vs Uniform** (same budget, samples concentrated where paths live);
**w22.2 Heuristic Calibration** (learned $h$ vs. true cost-to-go; admissibility violations
highlighted).
*[P]* `learn` crate with `candle`: a small conditional sampler trained from Ch. 11 roadmaps on
randomized Workbenches; learned heuristic plugged into Ch. 6 A\*; honest benchmarks.
*Sources: NEW — Ichter et al. 2018; Qureshi et al. 2019/2021; Janner et al. 2022; others to be
verified at authoring time.*

**23. Capstone: A Planning Stack from Sensor to Motion**
Everything, integrated and honest: Rusty navigates an unknown Apartment (sensor-based planning +
replanning on a Part IV belief), Reach executes a time-optimal collision-free trajectory on the
Workbench (Chs. 11–13 → 18 → 19), Hitch parks the trailer in the Lot (Chs. 20–21); the stack
mapped onto OMPL/MoveIt/Nav2; failure tour (blocked corridor, singular arm configuration, trailer
jackknife) and recovery; engineering retrospective.
*[C]* **w23.1 The Grand Plan** (full-page, three panels, every planner's internals inspectable);
**w23.2 Stack Anatomy** (block diagram with live message rates; turn a block off and watch).
*[P]* `capstone` crate wiring every module; the closing argument that the whole teaching stack runs
at real time in the browser.
*Sources: whole book; systems framing from OMPL/MoveIt/Nav2 architecture descriptions.*

---

### Appendices

- **A. Rust for Roboticists** — shared with the sister book (ownership for planner state, traits
  vs. inheritance, const generics for dimensions, the workspace, WASM builds).
- **B. Topology, Geometry & Rotations Reference Card** — sets (C App. B), topology and metric
  spaces (App. C), the implicit function theorem and curve tracing (App. D), representations of
  orientation (App. E), the §12.1 differential-geometry glossary: one page per concept.
- **C. Polyhedral Collision Geometry Reference** — C App. F in full: polygon representation,
  intersection tests, the star algorithm, $SE(2)$ C-obstacles, GJK — and what `parry2d` does with it.
- **D. Complexity & Completeness** — C App. G: running time, complexity classes, the three
  completeness notions, and the PSPACE-hardness of the generalized mover's problem.
- **E. Linear Systems & Stability Card** — C App. J.1–J.3 (state space, stability, LTI control);
  App. I is a pointer to the sister book's Ch. 2.
- **F. The Simulator & Widget Framework** — architecture reference for `sim`, `robots`, `collide`,
  and the shared widget chrome.

---

## 4. Baseline Source Map

### Reading the source

The baseline PDF is not committed (copyrighted; see README). Its extracted text, when present,
lives at `Resource/choset.txt` with `=== PAGE N ===` markers (PDF page numbers). Book-page →
PDF-page offsets drift (19 at Ch. 1, 14 by Ch. 12 and the appendices), so use this table:

| Choset chapter | PDF page | | Appendix | PDF page |
|---|---|---|---|---|
| 1 Introduction | 20 | | A Notation | 487 |
| 2 Bug Algorithms | 35 | | B Sets | 489 |
| 3 Configuration Space | 57 | | C Topology | 492 |
| 4 Potential Functions | 94 | | D Curve Tracing | 501 |
| 5 Roadmaps | 124 | | E Orientation | 503 |
| 6 Cell Decompositions | 177 | | F Polyhedral | 513 |
| 7 Sampling-Based | 213 | | G Complexity | 527 |
| 8 Kalman Filtering | 284 | | H Graph Search | 535 |
| 9 Bayesian Methods | 316 | | I Statistics | 561 |
| 10 Robot Dynamics | 363 | | J Linear Systems | 566 |
| 11 Trajectory Planning | 387 | | Bibliography | 578 |
| 12 Nonholonomic | 415 | | | |

### Choset section → chapter (every section assigned exactly once; "primary" owner listed)

| Choset | This book | Note |
|---|---|---|
| §1.1–1.3 | Ch. 1 | taxonomy, book overview, mathematical style |
| §2.1, §2.2, §2.3.1–2.3.3 | Ch. 3 | §2.3.2 sonar detail cross-linked to sister |
| §3.1, §3.2 (3.2.1–3.2.2), §3.3, §3.7 | Ch. 4 | |
| §3.4 (3.4.1–3.4.4), §3.5 (3.5.1), §3.6 | Ch. 5 | §3.6 kept short; App. E card carries detail |
| §3.8 | **split:** FK → Ch. 2; Jacobian & singularities → Ch. 4; $D\varphi$ revisited Ch. 5 | needed before Ch. 5 by §3.2.2 and §4.7.3 |
| §4.1–4.7 (4.3.1–4.3.2, 4.6.1–4.6.2, 4.7.1–4.7.3) | Ch. 7 | |
| §5.1 (5.1.1–5.1.2), §5.2 (5.2.1–5.2.5) | Ch. 8 | Morse definition box lives here |
| §5.3 (5.3.1–5.3.3), §5.4, §5.5 (5.5.1–5.5.2) | Ch. 9 | Lyapunov definition box lives here |
| §6.1, §6.2 (6.2.1–6.2.5), §6.3 | Ch. 10 | §6.3 as coda |
| §7.1 (7.1.1–7.1.4), §7.4 (7.4.1–7.4.3) | Ch. 11 | Lazy PRM (§7.1.4) labelled classical |
| §7.2 (7.2.1–7.2.3), §7.3 | Ch. 12 | |
| §7.5.1 | **preview** Ch. 14; **full** Ch. 21 | depends on Chs. 17/20; classical (LaValle–Kuffner) |
| §7.5.2, §7.5.3 | Ch. 14 | |
| §7.5.4–7.5.6 | Ch. 14 (pointer box) | |
| §8.1, §8.2 (8.2.1–8.2.7), §8.3 (8.3.1–8.3.3), §8.4 (8.4.1–8.4.2) | Ch. 15 | compressed |
| §9.1 (9.1.1–9.1.5), §9.2 (9.2.1–9.2.2) | Ch. 16 | compressed |
| §10.1–10.4 (10.4.1–10.4.2) | Ch. 17 | |
| §11.1, §11.2 (11.2.1–11.2.2), §11.3.3 | Ch. 18 | |
| §11.3.1, §11.3.2 | Ch. 19 | as the historical opening |
| §12.1 (12.1.1–12.1.3), §12.2, §12.3 (12.3.1–12.3.2), §12.4.1, §12.4.3 | Ch. 20 | |
| §12.4.2, §12.5 (12.5.1–12.5.8) | Ch. 21 | §12.5.3 cross-links Ch. 19 |
| App. A | this file, §2 notation table | |
| App. B | Appendix B | |
| App. C (C.1–C.5) | Ch. 5 + Appendix B | |
| App. D (D.1–D.2) | Ch. 3 (owned artifact) + Appendix B | |
| App. E (E.1–E.4) | Appendix B; short section in Ch. 5 | duplicates sister Ch. 3 |
| App. F (F.1–F.5) | Ch. 4 + Appendix C | F.5 is Ch. 2's black box |
| App. G (G.1–G.3) | Ch. 6 + Appendix D | |
| App. H (H.1, H.2.1–H.2.5, H.3, H.4) | Ch. 6 | |
| App. I (I.1–I.3) | pointer to sister Ch. 2 (Appendix E note) | dropped |
| App. J.1, J.4, J.5 (J.5.1–J.5.2) | Ch. 15 | |
| App. J.2, J.3 | Appendix E | Ch. 9 carries a Lyapunov definition box |

### Modernization set (every item **verified at authoring time** before it is cited)

| Topic | Canonical sources (to verify) |
|---|---|
| Replanning search | Koenig & Likhachev, D\* Lite (AAAI 2002; IEEE T-RO 2005); Likhachev, Gordon & Thrun, ARA\* (NIPS 2003) |
| Tree planners | Şucan & Kavraki, KPIECE (WAFR 2008); Kuffner & LaValle, RRT-Connect (ICRA 2000 — *classical*) |
| Optimal sampling | Karaman & Frazzoli (IJRR 30(7), 2011); Solovey, Janson, Schmerling, Frazzoli & Pavone (ICRA 2020, radius correction); Gammell, Srinivasa & Barfoot, Informed RRT\* (IROS 2014), BIT\* (ICRA 2015; IJRR 2020); Janson, Schmerling, Clark & Pavone, FMT\* (IJRR 34(7), 2015) |
| Many bodies | Sharon, Stern, Felner & Sturtevant, CBS (AIJ 219, 2015); Garrett et al., TAMP survey (Annu. Rev. Control Robot. Auton. Syst. 4, 2021); Şucan, Moll & Kavraki, OMPL (IEEE RAM 19(4), 2012); MoveIt (canonical paper **to verify**) |
| Time scaling | Pham & Pham, TOPP-RA (IEEE T-RO 34(3), 2018); Bobrow, Dubowsky & Gibson 1985 and Shin & McKay 1985 (classical, in Choset) |
| Trajectory optimization | Ratliff, Zucker, Bagnell & Srinivasa, CHOMP (ICRA 2009); Zucker et al. (IJRR 2013); Kalakrishnan et al., STOMP (ICRA 2011); Schulman et al., TrajOpt (RSS 2013; IJRR 2014); Li & Todorov, iLQR (ICINCO 2004); Jacobson & Mayne, DDP (1970); Tassa, Erez & Todorov (IROS 2012) |
| Nonholonomic planning | Dolgov, Thrun, Montemerlo & Diebel, hybrid A\* (IJRR 29(5), 2010); Pivtoraiko, Knepper & Kelly, state lattices (J. Field Robotics 26(3), 2009); Webb & van den Berg, Kinodynamic RRT\* (ICRA 2013); LaValle & Kuffner, kinodynamic RRT (IJRR 2001 — *classical*) |
| Learning | Ichter, Harrison & Pavone, learned sampling (ICRA 2018); Qureshi et al., MPNet (ICRA 2019; T-RO 2021); Bhardwaj, Choudhury & Scherer (CoRL 2017) and Choudhury et al. (IJRR 2018) learned heuristics (**to verify**); Janner et al., Diffuser (ICML 2022); Carvalho et al., Motion Planning Diffusion (**venue to verify**) |
| Geometry | Solà, Deray & Atchuthan, micro-Lie theory (arXiv 1812.01537) |
| Sister book | *Probabilistic Robotics via Rust* — Ch. 3 (Lie groups), Ch. 4 (Rusty/sim), Chs. 5–8 (filters), Chs. 10, 12–13, 17 (sensor models, MCL, grids, FastSLAM), Ch. 19 (ESDF), Ch. 20 (planning preview), Ch. 21 (MDPs), Ch. 23 (MPPI) |

Already in Choset's bibliography and therefore **not** "modern": Lazy PRM, RRT-Connect,
kinodynamic RRT, Reeds–Shepp, the bridge test, OBPRM, D\*.

---

## 5. Deliverable Map

- `TOC.md` — this file: the book's contract.
- `Chapter-01.md` … `Chapter-23.md` — per-chapter design docs (storyline, F/C/P plan, widget
  manifest, Rust module plan, exercises, modernization notes) following the shared 8-section
  template of the sister book.
- `CLAUDE.md` — the development guide: conventions, ownership, source offsets, and the definition
  of done for a chapter.
- `README.md` — what the book is, how it relates to the sister volume, and what is not yet built.
- *Not yet present (follow-on work):* `web/` (forked from the sister book), `pr-latex/`.
