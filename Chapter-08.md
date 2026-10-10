# Chapter 8 — Roadmaps I: Visibility Graphs and the Generalized Voronoi Diagram

> Part II — Classical Planners: Search, Potentials, Roadmaps, Cells · Estimated length: 10 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Chapters 6 and 7 answered one query at a time. This chapter builds something to keep: a network of
one-dimensional curves in $\mathcal{Q}_{free}$ that captures everything a planner needs, so that
every later query is "get on, ride, get off." The "aha" is that *two very different curves
qualify*. The **visibility graph** strings obstacle vertices together along lines of sight and
contains the shortest path — one that grazes every corner. The **generalized Voronoi diagram** is
the set of points equally far from the two nearest obstacles — the path that stays as far from
everything as it can. One optimizes length, the other clearance; both are roadmaps because both
satisfy the three-word contract: accessibility, connectivity, departability. Proving it for the GVD
brings two ideas that run through the rest of Part II: a **deformation retraction** (the free space
melts onto its skeleton without tearing, so loops survive) and the **preimage theorem** (one
equation in the plane cuts out a curve wherever its gradient does not vanish). The second comes
with the Morse vocabulary that Chapters 9 and 10 use without re-introducing. Reader path: Hook →
C (play) → F (rigor) → P (Rust) → Integration lab → Exercises.

**Story line**
1. **Hook** — one Apartment query solved three ways: A\* on the lattice (Ch. 6), the visibility
   path (hugging corners), the GVD path (down the middle); length and clearance side by side.
   Rusty, being a disc, prefers the second — or does it?
2. **Play (C)** — w8.1 Rotational Sweep: a lighthouse beam turns, the visible list updates; w8.2
   Voronoi Carver: brushfire fronts collide into ridges, the exact GVD overlaid.
3. **Formalize (F)** — roadmap (C Def. 5.0.2); visibility graph and its reduced form; the GVD via
   $d_i - d_j = 0$; accessibility by gradient ascent; deformation retractions and $\pi_1$; the
   preimage theorem with the Morse definition box; $SS_{ij}$; sensor, polygonal, grid construction.
4. **Algorithm** — C Alg. 5 rotational plane sweep; exact segment Voronoi (delegated); brushfire
   ridges (Ch. 7); retraction of a query onto the roadmap; A\* on the roadmap (Ch. 6).
5. **Implementation (P)** — the `roadmap` crate: `visibility` and `gvd` modules, a `Roadmap`
   graph over `petgraph`, `retract`, and `query`.
6. **Integration lab** — Rusty in the Apartment: both roadmaps built once, fifty seeded queries
   each, length and clearance histograms; Reach: the GVD of its $T^2$ C-space from the Ch. 4 raster.
7. **Close** — the planar GVD is one-dimensional but the $\mathbb{R}^3$ GVD is sheets: the
   cliffhanger Chapter 9 resolves with the GVG.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (worlds, `collide`), Ch. 3 (`trace_curve`, $D$ and $\nabla D$ from range
  data — imported for §5.2.5), Ch. 4 (polygonal C-obstacles for disc-Rusty, where the visibility
  graph lives; the $T^2$ raster for Reach), Ch. 5 (manifolds, tangent spaces, $D\varphi$ — needed
  for the preimage theorem), Ch. 6 (A\* on the roadmap), Ch. 7 (brushfire back pointers — grid GVD).
- **Feeds into:** Ch. 9 (GVG, HGVG, Canny — built on the preimage theorem and the Morse box defined
  here), Ch. 10 (Morse and brushfire decompositions), Ch. 11 (medial-axis sampling is "sample near
  the GVD"; the roadmap contract reappears as PRM's), Ch. 21 (Voronoi fields in hybrid A\*, verify).
- **Baseline sources:** C Ch. 5 — intro and Def. 5.0.2 (b. 107–110); §5.1.1 visibility graph,
  supporting/separating lines, reduced graph, local convexity [C refs. 262, 298, 294] (b. 110–113);
  §5.1.2 construction, Alg. 5, Fig. 5.8, Table 5.1, $O(n^2 \log n)$, general position [C ref. 124]
  (b. 113–117); §5.2 intro (b. 117–118); §5.2.1 Eq. (5.1), $S_{ij}$, $F_{ij}$, meet and boundary
  points (b. 118–119); §5.2.2 Eq. (5.2), Lemma 5.2.1, line-of-sight departability (b. 119–120);
  §5.2.3 retracts, homotopy, deformation retraction, $\pi_1$ [C refs. 410, 207, 340] (b. 121–123);
  §5.2.4 $SS_{ij}$, Figs. 5.10–5.11, Theorem 5.2.2 preimage theorem [C ref. 173] (b. 123–126);
  §5.2.5 sensor-based, polygonal (edge/node counts [C ref. 359]), brushfire (b. 126–129). Sister
  book: Ch. 19 (`…/chapters/ch19-map-representations`, ESDF ridges), Ch. 20
  (`…/chapters/ch20-motion-planning`, visibility graph as exercise). Modernization set: none
  required; `boostvoronoi` (to verify) for exact segment Voronoi.

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped):

| Symbol | Meaning |
|---|---|
| $RM$ | a roadmap: union of one-dimensional curves in $\mathcal{Q}_{free}$ |
| $v_i$, $e_{ij}$ | visibility-graph nodes (obstacle vertices, start, goal) and line-of-sight edges |
| $l$, $\alpha_i$, $E$, $S$; $n$, $N$ | sweep half-line, vertex angles, sorted angle list, active edge list (C Alg. 5); number of obstacles, of obstacle vertices |
| $F_i$ | generalized Voronoi region of $\mathcal{QO}_i$ (C Eq. 5.1) |
| $S_{ij}$, $SS_{ij}$, $F_{ij}$ | two-equidistant surface; its surjective subset ($\nabla d_i \ne \nabla d_j$); two-equidistant face |
| $F_{ijk}$ | meet point: equidistant to three closest obstacles; boundary point: $D = 0$ |
| $H(x,t)$, $f \sim g$, $\pi_1(X, x_0)$ | homotopy; homotopic maps; first fundamental group (loop classes through $x_0$) |
| $G^{-1}(n)$, $DG$, $\Sigma(G)$, $T_m M$ | preimage, differential, critical set, tangent space of $M$ at $m$ |

**Definitions** (display boxes):
- *Roadmap* (C Def. 5.0.2): a union of one-dimensional curves $RM$ such that for every connectable
  $q_{start}, q_{goal}$: **accessibility** (a path from $q_{start}$ to some $q'_{start} \in RM$),
  **departability** (from some $q'_{goal} \in RM$ to $q_{goal}$), **connectivity** (a path in $RM$).
- *Visibility graph* (C §5.1.1): nodes are $q_{start}$, $q_{goal}$, and all C-obstacle vertices;
  $e_{ij} \ne \emptyset \iff s v_i + (1-s) v_j \in \mathrm{cl}(\mathcal{Q}_{free})\ \forall s \in [0,1]$.
- *Supporting / separating line; reduced visibility graph; locally convex* (C §5.1.1): a line
  tangent to two obstacles with both on one side / on opposite sides (nonsmooth tangency at
  vertices); the reduced graph keeps only edges on such lines; a boundary is locally convex at
  $\bar c$ if the tangent hyperplane there separates nearby boundary points.
- *Generalized Voronoi region, GVD* (C §5.2.1): $F_i = \{q \in \mathcal{Q}_{free} : d_i(q) \le
  d_h(q)\ \forall h \ne i\}$; $\mathrm{GVD} = \bigcup_{i}\bigcup_{j} F_{ij}$, $F_{ij} = \{q \in
  SS_{ij} : d_i(q) \le d_h(q)\ \forall h\}$ (after the §5.2.4 refinement).
- *Retraction, deformation retraction* (C §5.2.3): continuous $f : X \to A \subset X$ with
  $f|_A = \mathrm{id}$; a homotopy $H$ with $H(\cdot,0) = \mathrm{id}$, $H(\cdot,1) \in A$, $H(a,t) = a$ on $A$.
- *Regular value, critical point, critical value* (C §5.2.4): $n$ is regular if $DG(m)$ is
  surjective for all $m \in G^{-1}(n)$; a critical point has non-surjective $DG$, its image a
  critical value.
- **Morse function (definition box — introduced here, imported by Chs. 9–10):** a smooth
  real-valued $f$ all of whose critical points are *nondegenerate* (nonsingular Hessian). Such
  critical points are isolated, each has an *index* (number of negative Hessian eigenvalues:
  0 minimum, $\dim$ maximum, otherwise saddle), and — the fact Chs. 9–10 lean on — the topology of
  the level sets $f^{-1}(\lambda)$ changes only as $\lambda$ crosses a critical value [C ref. 315].
  Ch. 7's navigation functions are Morse by definition; $D$ is *not* smooth, so Ch. 9 adds the
  generalized gradient.

**Derivations / theorems:**

1. **The visibility graph contains a shortest path** (C §5.1.1; connectivity is C Ch. 5 Problem 1).
   *Statement:* in a polygonal planar $\mathcal{Q}_{free}$, a Euclidean-shortest path is a polyline
   whose interior vertices are obstacle vertices, hence a path in the visibility graph.
   *Sketch (4 steps):* (i) a shortest path is locally shortest, so straight in the open free space;
   (ii) it can bend only on $\partial\mathcal{Q}_{free}$, and a bend on an edge's interior could be
   shortcut; (iii) so bends occur only at vertices; (iv) consecutive bend points see each other.
   *Collapsible:* the connectivity proof; why the 3D visibility graph misses shortest paths
   (C §5.1.1); taut paths use only supporting/separating lines (C §5.1.1, Problem 21).
2. **Rotational plane sweep is correct and runs in $O(n^2 \log n)$** (C §5.1.2, Alg. 5).
   *Statement:* sweeping a half-line $l$ from $v$ through $[0, 2\pi)$ with the crossing edges kept
   sorted by distance, $v_i$ is visible iff $\overline{v v_i}$ misses the closest edge in $S$ and $l$
   does not enter the obstacle at $v$. *Sketch (4 steps):* (i) the crossed-edge set changes only at
   vertex angles, so pausing at the $\alpha_i$ loses nothing; (ii) the nearest crossed edge is the
   only possible occluder; (iii) each event does $O(1)$ balanced-tree updates, $O(\log n)$; (iv) $n$
   events per vertex, $n$ vertices. *Collapsible:* C Table 5.1 for Fig. 5.8 (initial $S = \{E_4,
   E_2, E_8, E_6\}$; edges added $(v,v_4)$, $(v,v_8)$, $(v,v_1)$); general position [C ref. 124].
3. **Accessibility of the GVD** (C Lemma 5.2.1). *Statement:* in an obstacle-bounded environment,
   gradient ascent $\dot c = \nabla D(c)$ from any $q \in \mathcal{Q}_{free}$ reaches the GVD.
   *Sketch (3 steps):* (i) with $\mathcal{QO}_i$ the unique closest obstacle at $q$, $D = d_i$ nearby
   and the flow is $\nabla d_i$; (ii) $d_i$ increases while the environment is bounded, so some
   $d_j$ must stop exceeding it; (iii) continuity gives $\bar t$ with $d_i(c(\bar t)) = d_j(c(\bar t))$.
   *Collapsible:* departability as accessibility reversed; every free point sees a GVD point (C §5.2.2).
4. **The GVD is a deformation retract, hence connected, hence a roadmap** (C §5.2.2–5.2.3).
   *Statement:* the gradient-ascent map $RM(q)$ is a retraction homotopic to the identity; each
   component of $\mathcal{Q}_{free}$ has exactly one GVD component, and $\pi_1$ is preserved.
   *Sketch (5 steps):* (i) $RM(q) = q$ on the GVD, else the ascent endpoint; (ii) $RM$ is continuous
   [C ref. 340]; (iii) $H(q,t) = c_q(t)$ along the ascent is a homotopy from $\mathrm{id}$ to $RM$
   fixing the GVD; (iv) continuous images of connected sets are connected; (v) deformation retracts
   preserve $\pi_1$, so the GVD has the free space's loop classes. *Collapsible:* retract vs.
   deformation retract (a point is a retract of the doughnut, not a deformation retract).
5. **The planar GVD consists of one-dimensional manifolds** (C §5.2.4, Theorem 5.2.2).
   *Statement (preimage theorem):* for smooth $G : M \to N$ and a regular value $n$, $G^{-1}(n)$ is
   a closed submanifold of dimension $\dim M - \dim N$ with $T_m G^{-1}(n) = \ker DG(m)$; for
   $G = d_i - d_j : \mathcal{Q} \to \mathbb{R}$, $SS_{ij} \subset G^{-1}(0)$ is one-dimensional in the
   plane. *Sketch (4 steps):* (i) $DG = (\nabla d_i - \nabla d_j)^\top$; (ii) $DG$ is surjective iff
   $\nabla d_i \ne \nabla d_j$; (iii) that is exactly the condition defining $SS_{ij}$ — the word
   *surjective* in its name; (iv) $F_{ij} \subset SS_{ij}$ is a submanifold. *Collapsible:* C's
   warm-up $f(x,y) = x^2 + y^2$, $f^{-1}(91{,}538)$ a circle; why nonconvex obstacles force the
   refinement (the two cuts of C Fig. 5.10 give different $S_{ij}$, the same $SS_{ij}$); a robot at
   $q_2$ outside the concavity sees one local minimum of $\rho$, not two (C Fig. 5.11); a GVD edge's
   tangent is perpendicular to the chord joining the two closest points (C Fig. 5.13).
6. **Size and construction of the polygonal GVD** (C §5.2.5). *Statement:* with $n$ obstacles and
   $N$ vertices the GVD has between $\tfrac{3(n+1)}{2}$ and $6N + 3n - 3$ edges and between
   $\tfrac{n+5}{2}$ and $4N - n - 2$ nodes [C ref. 359]; edges are straight (vertex–vertex,
   edge–edge) or parabolic (vertex–edge). *Sketch (3 steps):* (i) point–point and line–line
   bisectors are lines, point–line a parabola; (ii) the plane partitions by closest feature pair
   (C Fig. 5.14); (iii) the counts follow from Euler's formula on the planar GVD graph. *Collapsible:*
   brushfire — a pixel reached by two fronts with different back pointers is a GVD pixel (C §5.2.5),
   i.e., Ch. 7's `ridge_cells`.

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| Brute-force visibility (C §5.1.2) | `(polys, V) -> Graph` | $O(n^3)$ |
| C Alg. 5 Rotational Plane Sweep | `(polys, v) -> Vec<visible v_i>` | $O(n \log n)$ per vertex, $O(n^2 \log n)$ total |
| Reduced visibility graph (C §5.1.1) | `(Graph) -> Graph` | filter by supporting/separating test, $O(E)$ |
| Exact segment GVD (delegated; `boostvoronoi`, to verify) | `(segments) -> Gvd` | $O(N \log N)$ |
| Brushfire-ridge GVD (C §5.2.5; Ch. 7 `brushfire`) | `(raster) -> ridge cells` | $O(\#cells)$ |
| GVD retraction (C Eq. 5.2 / Lemma 5.2.1) | `(q) -> q' on GVD` | gradient ascent of $D$ via Ch. 3/4 distance |
| Sensor-based GVD edge tracing (C §5.2.5) | `trace_curve(G = d_i - d_j, DG)` (Ch. 3) | predictor–corrector per step |
| Roadmap query | `retract(q_s), retract(q_g), astar (Ch. 6)` | $O(E \log V)$ on the roadmap graph |

**Numeric micro-example** (the chapter's contract with the unit test). A Workbench slab with two
point obstacles (pegs) $P_1 = (0,0)$, $P_2 = (4,0)$ and a straight wall $W$ along $y = 4$.
- *Edge $P_1 | P_2$*: the bisector $x = 2$. *Edge $P_1 | W$*: $\sqrt{x^2 + y^2} = 4 - y \iff
  y = 2 - x^2/8$ (a parabola). *Edge $P_2 | W$*: by symmetry $y = 2 - (x-4)^2/8$.
- *Meet point $F_{P_1 P_2 W}$*: on $x = 2$, $\sqrt{4 + y^2} = 4 - y \Rightarrow 8y = 12 \Rightarrow
  y = 1.5$: the meet point is $(2, 1.5)$ with clearance $D = 2.5$ (check: $\sqrt{4 + 2.25} = 2.5 =
  4 - 1.5$); both parabolas pass through it ($2 - 4/8 = 1.5$).
- *Surviving pieces*: the vertical edge is the ray $x = 2$, $y < 1.5$ (above the meet point the wall
  is closer than either peg); the parabolas survive for $x \le 2$ and $x \ge 2$ respectively.
- *Tangent at the meet point along $P_1|W$*: $dy/dx = -x/4 = -0.5$, perpendicular to the chord
  from $P_1$ to the wall foot $(2,4)$, as C Fig. 5.13 says ($(1, -0.5) \cdot (2, 4) = 0$).
- *Grid check*: brushfire on a $0.25$-resolution raster of $[-2,6] \times [-1,4]$ yields a ridge
  cell containing $(2, 1.5)$ with three distinct back-pointer obstacles among its 8-neighbors.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, carried end to end: **"the highway system"** (C Fig. 5.2) — get on, ride, get
off; two highway designers with opposite philosophies. Colors per the book code: start green, goal
red, obstacles slate, C-obstacles amber hatched (the visibility graph lives on them), roadmap edges
in the graph color, the query path purple, Rusty orange.

- **Widget w8.1: Rotational Sweep** — type: interactive animation. C Fig. 5.8's two rectangles
  and a chosen vertex $v$; a half-line sweeps; the active list $S$ and angle list $E$ render as
  live tables (C Table 5.1). **Manipulates:** headline — which vertex $v$ is the lighthouse (click
  any vertex); disclosure: scrub the sweep angle, add a polygon, reduced-graph toggle, seed re-roll.
  **Observes:** edges entering and leaving $S$ at each event; a visible vertex flashing as its edge
  is added; the full graph and A\* shortest path at the end; needless edges fading under the reduced
  toggle. **Misconception killed:** "visibility needs $O(n)$ tests per pair" — the sweep makes the
  nearest crossed edge the only one to test.
- **Widget w8.2: Voronoi Carver** — type: wasm-sim. The §3 three-obstacle slab, then an Apartment
  room. Brushfire fronts grow from the obstacles (w7.4 colors), ridge cells light up where fronts
  meet, and an "exact" layer overlays the `boostvoronoi` GVD. **Manipulates:** headline — raster
  resolution ($1$ m down to $5$ cm); disclosure: drag the pegs and wall, layer toggles, 4/8
  connectivity, seed re-roll of obstacle placement. **Observes:** the meet point $(2,1.5)$ marked;
  the ridge converging to the exact curves as resolution rises; jagged staircase artifacts of grid
  ridges (C Problem 14's warning); the parabola–line structure of the exact diagram. **Misconception
  killed:** "the Voronoi diagram is made of straight lines" — between a vertex and an edge it is a parabola.
- **Widget w8.3: Retract Candy** — type: interactive animation. The Apartment's free space melts:
  $H(q,t)$, $t \in [0,1]$, animates every point along its gradient ascent onto the GVD.
  **Manipulates:** headline — the homotopy time $t$ (scrub); disclosure: drop a loop and watch it
  retract; toggle "retract to a point" for a retraction that is *not* a deformation retraction.
  **Observes:** loops around a kitchen island stay loops (the hole survives); a loop in an open room
  shrinks to a point; the counts "loop classes in $\mathcal{Q}_{free}$" and "in GVD" always agree.
  **Misconception killed:** "a skeleton is any thinner version of the space" — only a *deformation*
  retract keeps the holes.

Dashboard layout: w8.2 full-width atop the C section (the chapter's signature image); w8.1
half-width beside §5.1; w8.3 half-width beside the deformation-retract derivation; a sticky panel
ticks the three roadmap properties as widgets demonstrate them. All autoplay seeded defaults and
degrade to static SVG.

## 5. Practical (P) — Rust Implementation

**Crates:** `petgraph` 0.8 (`UnGraph` roadmap, `Point2` nodes, length edges), `parry2d` 0.30
(`parry2d-f64`; segment intersection, closest points for $\nabla D$, via the Ch. 2 `collide`
trait), `boostvoronoi` (to verify; exact Voronoi of points and segments — this chapter's one
delegated kernel, per the hand-rolling policy), `nalgebra` 0.35, `rand` 0.9 (`SmallRng`), plus
`search` (Ch. 6), `potential::brushfire` (Ch. 7), `bugs::trace_curve` (Ch. 3), `cspace` (Ch. 4).

**Module plan:**
- `crates/roadmap/` — **introduced here; `gvg` (Ch. 9) is a submodule added later.** `src/lib.rs`
  (`Roadmap` and the three-property contract as a trait), `src/visibility.rs` (brute force,
  `rotational_sweep` = C Alg. 5, `reduced`), `src/gvd/mod.rs` (`Gvd`, `GvdEdge::{Line, Parabola}`,
  `MeetPoint`), `src/gvd/exact.rs` (`boostvoronoi` adapter), `src/gvd/grid.rs` (Ch. 7 ridge cells
  → thinned polyline graph), `src/gvd/retract.rs` (gradient ascent of $D$; `trace_edge` via
  `trace_curve` on $G = d_i - d_j$), `src/query.rs` (`retract` + Ch. 6 `astar`),
  `examples/{sweep_table, three_obstacle_gvd, apartment_roadmaps}.rs`.
- `web/lib/roadmap/` — TypeScript port for w8.1–w8.3; `__checks__.ts` pins the meet point.

**Key types & signatures:**

```rust
use nalgebra::Point2; use petgraph::graph::{UnGraph, NodeIndex};

/// A graph embedded in Q_free plus the access map of C Def. 5.0.2; edge geometry is sampled.
pub struct Roadmap { pub graph: UnGraph<Point2<f64>, f64>, pub polylines: Vec<Vec<Point2<f64>>> }
pub trait Access { fn access(&self, q: Point2<f64>) -> Option<(NodeIndex, Vec<Point2<f64>>)>; }

// src/visibility.rs
/// C Alg. 5 for one vertex; `S` is a BTreeMap keyed by distance along the sweep line — the
/// balanced tree the O(n log n) bound assumes.
pub fn rotational_sweep(polys: &[cspace::Polygon], v: Point2<f64>) -> Vec<Point2<f64>>;
pub fn visibility_graph(polys: &[cspace::Polygon], extra: &[Point2<f64>]) -> Roadmap;
/// Keeps only edges on supporting or separating lines (C §5.1.1).
pub fn reduced(polys: &[cspace::Polygon], g: &Roadmap) -> Roadmap;

// src/gvd/mod.rs
pub enum GvdEdge { Line { a: Point2<f64>, b: Point2<f64> }, Parabola { focus: Point2<f64>, directrix: (Point2<f64>, Point2<f64>), t: (f64, f64) } }
pub struct MeetPoint { pub q: Point2<f64>, pub clearance: f64, pub obstacles: [usize; 3] }
pub struct Gvd { pub edges: Vec<(GvdEdge, [usize; 2])>, pub meet_points: Vec<MeetPoint>, pub boundary_points: Vec<Point2<f64>> }
/// Exact GVD of points and segments (`boostvoronoi`, to verify); edges piercing obstacles or on
/// non-surjective parts of S_ij (C §5.2.4) are discarded here.
pub fn exact(sites: &[cspace::Segment], bounds: cspace::Aabb) -> Gvd;
/// Grid GVD: Ch. 7 `brushfire` ridge cells (two distinct back pointers), thinned to a graph.
pub fn from_brushfire(dist: &potential::DistGrid) -> Roadmap;
// src/gvd/retract.rs
/// C Eq. (5.2): follow ∇D until two obstacles tie (Lemma 5.2.1 guarantees termination in a
/// bounded world); `dist` supplies D and ∇D from Ch. 4 queries or a Ch. 3 range scan.
pub fn retract(dist: &dyn cspace::DistanceQuery<2>, q: Point2<f64>, step: f64) -> Vec<Point2<f64>>;
/// Sensor-based edge following: Ch. 3's predictor–corrector on G = d_i - d_j (C §5.2.5).
pub fn trace_edge(dist: &dyn cspace::DistanceQuery<2>, q0: Point2<f64>, step: f64) -> (Vec<Point2<f64>>, EdgeEnd);
pub enum EdgeEnd { MeetPoint(MeetPoint), BoundaryPoint(Point2<f64>) }
// src/query.rs
pub struct QueryResult { pub path: Vec<Point2<f64>>, pub length: f64, pub min_clearance: f64 }
pub fn query(rm: &Roadmap, acc: &dyn Access, q_s: Point2<f64>, q_g: Point2<f64>) -> Option<QueryResult>;
```

**Worked end-to-end example:** `cargo run --example three_obstacle_gvd -p roadmap` builds the two
pegs and the wall, calls `exact`, and prints `meet point (2.000, 1.500) clearance 2.500`, the
parabolas `y = 2 - x^2/8` and `y = 2 - (x-4)^2/8`, and the vertical ray; then rasterizes at
$0.25$ m, runs `from_brushfire`, and prints the ridge cell containing the meet point.
`#[test] fn meet_point_is_2_1p5()` asserts the exact meet point to $10^{-9}$, the clearance, the
tangent slope $-0.5$ along $P_1|W$, and that the grid ridge passes within one cell of $(2,1.5)$.
`cargo run --example sweep_table` reproduces C Table 5.1 for Fig. 5.8 and asserts the added edges
are exactly $(v,v_4), (v,v_8), (v,v_1)$; `#[test] fn sweep_equals_brute_force()` compares
`rotational_sweep` with the $O(n^3)$ method on 100 seeded polygon sets. `cargo run --example
apartment_roadmaps` builds both roadmaps for disc-Rusty and answers 50 seeded queries, printing
mean length and mean minimum clearance (visibility shorter, GVD safer — logged, not pinned).

**Runnable artifact:** w8.1 and w8.3 are `web/lib/roadmap/` ports of `rotational_sweep` and
`retract`; w8.2 ports `from_brushfire` and renders the exact GVD precomputed by the Rust crate (the
`boostvoronoi` kernel is not ported; its output ships as data, checked against the meet point). The
Integration lab runs the fifty-query comparison live in the Apartment and builds the GVD of
Reach's $T^2$ C-space from the Ch. 4 raster, drawn on the unwrapped torus.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w8.1 | Rotational Sweep | interactive animation | `roadmap::visibility` → `web/lib/roadmap/`, `search` | pick vertex, scrub angle, add polygon, reduced toggle, seed re-roll | sweep maintains only the nearest occluder; reduced graph loses nothing |
| w8.2 | Voronoi Carver | wasm-sim | `roadmap::gvd`, `potential::brushfire` → TS port; exact GVD shipped as data | resolution slider, drag obstacles, layer toggles, seed re-roll | ridges are the GVD; parabolas, not lines; grid artifacts |
| w8.3 | Retract Candy | interactive animation | `roadmap::gvd::retract` → TS port | homotopy scrub, drop a loop, retraction-to-point toggle | deformation retracts preserve loops; retracts need not |
| — | Three-ways hook | animation (autoplay) | `search`, `roadmap`, `sim` | none (replay) | length vs. clearance; both are roadmaps |
| f8.4 | Bisector zoo | static-svg | build-time SVG | hover | point–point, line–line lines; point–line parabola (C Fig. 5.14) |

## 7. Exercises & Extensions

1. **(F)** Prove the visibility graph is connected within a connected component of
   $\mathcal{Q}_{free}$ (C Ch. 5 Problem 1), and show by example that it misses the shortest path
   in $\mathbb{R}^3$ (Problem 2).
2. **(F)** Use the preimage theorem to state for which $c$ the set $f^{-1}(c)$, $f(x,y,z) = x^2 +
   y^2 - z^2$, is a manifold, its dimension, and when it is connected (C Problem 18); explain why
   $c = 0$ fails and what the failure has to do with $SS_{ij}$.
3. **(C, predict-then-verify)** In w8.2, move the wall from $y = 4$ to $y = 6$. Predict the new meet
   point by hand ($\sqrt{4 + y^2} = 6 - y$) before releasing; verify, then predict the vertical edge
   as the wall goes to infinity.
4. **(C)** In w8.3 drop a loop around the kitchen island and one inside an empty room; predict which
   survives the retraction and why the "loop classes" counters never disagree.
5. **(P)** Implement `reduced` for nonconvex polygons using the local-convexity test of C §5.1.1,
   with a test that shortest-path length on the reduced and full graphs agree for 100 seeded worlds.
6. **(P, stretch)** Implement the sensor-based GVD construction of C §5.2.5 end to end: Rusty with
   the Ch. 3 range sensor accesses the GVD by `retract`, traces edges with `trace_edge`, detects meet
   points as "a sudden change in one of the two closest obstacles," builds the graph incrementally,
   and compares its edge count with `exact` on the Apartment.

## 8. Modernization Notes

- **Kept from Choset, deliberately:** the roadmap contract (Def. 5.0.2) as the organizing idea of
  Chs. 8–11; the visibility graph with its sweep algorithm and C Table 5.1 trace; the GVD developed
  *mathematically* — deformation retractions, $\pi_1$, the preimage theorem — since that machinery
  lets Chs. 9–10 generalize to $\mathbb{R}^3$ and to coverage; the surjective refinement $SS_{ij}$
  with its sensor interpretation; the three construction methods with the edge-count bounds
  [C ref. 359].
- **Added beyond 2005:** exact segment Voronoi delegated to a crate (`boostvoronoi`, to verify),
  shown *beside* the brushfire approximation with a resolution slider so the reader sees it
  converge; the Morse definition box (index, level-set topology) as the book's single introduction;
  the GVD as the planar medial axis, with forward links to medial-axis sampling (Ch. 11) and
  Voronoi-field costs in hybrid A\* (Dolgov et al. 2010, verify); clearance-vs-length statistics
  over seeded query sets instead of a single figure.
- **Dropped/condensed:** curved-obstacle visibility graphs [C ref. 294] to a pointer; the freeway
  figure to a metaphor sentence; the opener's mapping taxonomy (topological / geometric / grid,
  occupancy grids [C ref. 142]) to a cross-link to the sister book's Chs. 13 and 19.
- **Honesty items the prose must keep:** (i) Choset leaves connectivity of the visibility graph as
  an exercise and so do we (theorem stated, proof in Exercise 1); (ii) the GVD maximizes clearance
  locally and can be much longer than the shortest path — the Integration lab prints both;
  (iii) grid ridges are jagged (C Problem 14) and the thinning in `from_brushfire` is heuristic;
  (iv) the exact kernel is delegated — one of the few places the book does not print the geometry
  it runs; it explains what the kernel does.
