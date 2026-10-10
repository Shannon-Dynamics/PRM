# Chapter 10 — Cell Decompositions and Coverage

> Part II — Classical Planners: Search, Potentials, Roadmaps, Cells · Estimated length: 9 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Every planner so far answered "how do I get *there*?" This chapter asks "how do I visit
*everywhere*?" — the question of a vacuum, a demining robot, a lawnmower. The "aha" is that one
object answers both: carve $\mathcal{Q}_{free}$ into **cells** simple enough to sweep with laps and
record which touch in an **adjacency graph**; planning is then graph search (Ch. 6), coverage an
exhaustive walk. The deeper "aha" is *where the cell walls go*. Trapezoidal decomposition puts one
at every polygon vertex — correct but over-chopped. Boustrophedon decomposition puts one only where
a sweeping slice changes connectivity: **critical points of the slice function on the obstacle
boundary**, the Morse vocabulary of Chapters 8–9 with a sensor reading (a surface normal parallel to
the sweep, which a range ring sees). Coverage thus becomes an online algorithm with a completeness
guarantee and a path-length bound linear in area, critical points, and perimeter. A coda carves
cells by *visibility* changes and finds an evader. Reader path: Hook → C (play) → F (rigor) → P
(Rust) → Integration lab → Exercises.

**Story line**
1. **Hook** — Rusty mows the Apartment living room with a raster scan and leaves a shadow under the
   sofa uncovered (autoplay of C Fig. 6.22's missed critical point). Coverage looked done.
2. **Play (C)** — w10.1 Boustrophedon Mower: cells split and merge at critical points, the Reeb
   graph grows, the path covers every cell; toggle trapezoidal and count the extra cells.
3. **Formalize (F)** — cell decompositions and adjacency graphs; the trapezoidal sweep; Morse
   decompositions (Def. 6.2.1), the Reeb graph, variable slices; sensor-based coverage via Lemma
   5.5.2 and the cycle algorithm; Euler's count and the path-length bound; the pursuit/evasion coda.
4. **Algorithm** — trapezoidal sweep; adjacency search; boustrophedon exhaustive walk; the cycle
   algorithm; information-graph search.
5. **Implementation (P)** — the `cells` crate: `trapezoidal`, `boustrophedon`, `coverage_path`,
   `ReebGraph`, `pursuit::information_graph`; a coverage-completeness test.
6. **Integration lab** — Workbench and Apartment: offline boustrophedon coverage, then online
   sensor-based coverage with the cycle algorithm; paths compared by length and missed area.
7. **Close** — Part II's scorecard (complete, exact, expensive); Part III's probabilistic bargain.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (polygonal worlds, Rusty's range ring, `collide`), Ch. 3 (boundary following
  and the curve tracer, used by §6.2.4), Ch. 4 (polygonal C-obstacles for disc-Rusty, on which
  decompositions are computed), Ch. 6 (search on adjacency graphs; DFS), Ch. 7 (brushfire and
  wave-front for §6.2.3), Ch. 8 (Morse box, GVD), Ch. 9 (Lemma 5.5.2; slices).
- **Feeds into:** Ch. 11 (the exact baseline PRM is measured against), Ch. 14 (adjacency graphs as
  task planning's discrete layer), Ch. 23 (exploring an unknown Apartment), sister Ch. 24.
- **Baseline sources:** C Ch. 6 intro (cell decomposition, adjacency graph, coverage as an
  exhaustive walk; b. 161–162); §6.1 trapezoidal decomposition, Figs. 6.1–6.4, list $L$ updates
  (Figs. 6.5–6.8), point location [C refs. 356, 124] (b. 162–168); §6.2 Morse cell decompositions,
  Fig. 6.9 (b. 168–169); §6.2.1 boustrophedon [C ref. 110], Fig. 6.10 (b. 169–170); §6.2.2 Def.
  6.2.1 [C ref. 12], slice intervals, $I^\star$, Fig. 6.11 [C refs. 96, 86] (b. 170–172); §6.2.3
  variable slices, brushfire [C ref. 262] and wave-front [C ref. 208] decompositions (b. 172–178);
  §6.2.4 sensor-based coverage, Reeb graph [C ref. 154], critical-point sensing, the cycle algorithm
  [C ref. 11] (b. 178–182); §6.2.5 Euler's formula $N_{ce} = N_{cp} + N_{ob} - 1$ and the
  path-length bound (b. 182–187); §6.3 pursuit/evasion, Suzuki & Yamashita [C ref. 403], LaValle &
  Guibas [C refs. 172, 273] (b. 187–194). Sister book: Ch. 24 (`…/chapters/ch24-exploration`).
  Modernization set: coverage pointers in §8, each (verify).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped):

| Symbol | Meaning |
|---|---|
| $c_i$ | a cell; the adjacency graph has a node per cell and an edge per shared boundary |
| $e_{lower}, e_{upper}$; $e_{LOWER}, e_{UPPER}$; $L$ | edges incident to the event vertex; free-space edges just below/above it; sweep line's sorted edge list (C §6.1) |
| $Q_\lambda$, $Q_{free\,\lambda}$, $Q^j_{free\,\lambda}$ | slice, its free part, its $j$-th connected interval |
| $I^\star$ | the set of slice intervals containing a critical point; cells are components of $\mathcal{Q}_{free} \setminus I^\star$ |
| $h : \mathcal{Q} \to \mathbb{R}$; $m$ | slice function ($h = x$ boustrophedon; $\sqrt{x^2+y^2}$ spiral; $\tan(y/x)$ spike; $|x|+|y|$ squarel; $D$ brushfire); implicit obstacle-boundary function, $\nabla h \parallel \nabla m$ at a critical point |
| $N_{cp}, N_{ce}, N_{ob}$; $\Delta$, $2r$, $P_{total}$ | numbers of critical points, cells, obstacles; enclosing-disk diameter, interlap spacing, total perimeter |
| $V(x)$, $B(x)$ | visibility polygon of $x$; binary vector of gap-edge labels (0 clear, 1 contaminated) |
| $H(\mathcal{W}_{free})$ | minimum number of pursuers that guarantee capture |

**Definitions** (display boxes):
- *Exact cell decomposition; adjacency graph* (C Ch. 6 intro): a partition of $\mathcal{Q}_{free}$
  into simple regions; a graph with a node per cell and an edge per shared boundary.
- *Coverage path* (C Ch. 6 intro): a path passing an effector of width $2r$ over every point of
  $\mathcal{Q}_{free}$; with cells, an exhaustive adjacency-graph walk plus a within-cell pattern.
- *Trapezoidal decomposition* (C §6.1): at every vertex in general position, draw upper and lower
  vertical extensions to the edges immediately above and below; cells are the resulting trapezoids.
- *Critical point (slice sense)* (C §6.2.1–6.2.2): where $Q_{free\,\lambda}$ changes connectivity —
  a vertex with extensions both up and down; smoothly, a critical point of
  $h|_{\partial\mathcal{Q}_{free}}$ (Ch. 8 Morse box, Ch. 9 Lemma 5.5.2).
- *Morse decomposition* (C Def. 6.2.1 [C ref. 12]): the exact cell decomposition whose cells are the
  components of $\mathcal{Q}_{free} \setminus I^\star$; boustrophedon is the case $h(x,y) = x$.
- *Reeb graph* (C §6.2.4 [C ref. 154]): the dual — nodes are critical points, edges are cells; built
  incrementally during sensor-based coverage.
- *Closing critical point* (C §6.2.4): the critical point whose discovery completes a cell.
- *Conservative region; information state* (C §6.3): a connected set on which $B(x)$ is constant;
  the pair $(x, B(x))$; contaminated / clear / recontaminated regions as in C §6.3.

**Derivations / theorems:**

1. **The trapezoidal sweep runs in $O(n \log n)$ with four event types** (C §6.1). *Statement:* with
   vertices sorted by $x$ and the intersected-edge list $L$ in a balanced tree, each event updates
   $L$ and finds $e_{LOWER}, e_{UPPER}$ in $O(\log n)$. *Sketch (4 steps):* (i) both incident edges
   to the right: insert $e_{lower}, e_{upper}$ between $e_{LOWER}, e_{UPPER}$; (ii) both to the
   left: delete them; (iii) one each way: replace the leaving edge by the arriving one (two
   sub-cases, C Fig. 6.4); (iv) extensions run from $v$ to $e_{LOWER}$ and $e_{UPPER}$ and exist
   exactly when free space lies on that side. *Collapsible:* the naive $O(n^2)$ alternative; slab
   point location (C §6.1 end, [C ref. 124]); general position and its perturbation fix; C Figs.
   6.5–6.8's trace $L: \emptyset \to \{e_8, e_{13}\} \to \{e_8, e_0, e_3, e_{13}\} \to \dots$.
2. **Midpoint–centroid paths are collision-free** (C §6.1). *Statement:* joining the midpoints of a
   trapezoid's extensions to its centroid, and start/goal to their cells' midpoints, gives a path in
   $\mathcal{Q}_{free}$ whenever the adjacency-graph path exists. *Sketch (3 steps):* (i) a
   trapezoid is convex; (ii) any two boundary points are joined by a segment inside it; (iii)
   consecutive cells share an extension. *Collapsible:* the adjacency graph as a Ch. 8 roadmap
   searched by Ch. 6's A\*.
3. **Between critical slices the free slice's topology is constant** (C §6.2.2; Ch. 8 Morse box; Ch.
   9 Derivation 5). *Statement:* if $h$ is Morse on $\partial\mathcal{Q}_{free}$, a diffeomorphism
   carries $Q_{free\,\lambda_1}$ to $Q_{free\,\lambda_2}$ for $\lambda_1, \lambda_2$ between
   adjacent critical values — slice intervals extend or contract, never split or merge. *Sketch (4
   steps):* (i) connectivity of $Q_{free\,\lambda}$ changes where the slice is tangent to
   $\partial\mathcal{Q}_{free}$; (ii) tangency is $\nabla h \parallel \nabla m$, i.e., $D(h,m)$
   loses rank (Lemma 5.5.2); (iii) those are the critical points of
   $h|_{\partial\mathcal{Q}_{free}}$; (iv) Morse theory [C ref. 315] gives constant topology between
   adjacent critical values. *Collapsible:* critical points lie at slice-interval endpoints, never
   inside; nonsmooth $h$ ($|x|+|y|$, $D$) via Ch. 9's generalized gradient; degenerate (rectilinear)
   critical points [C ref. 86].
4. **Critical points are sensed as "surface normal parallel to the sweep direction"** (C §6.2.4).
   *Statement:* while boundary-following, Rusty detects a critical point when its range ring's
   global minimum points along $\pm\nabla h$ (for $h = x$, straight ahead or behind). *Sketch (3
   steps):* (i) the closest obstacle point's direction is $\mp\nabla m$ (Ch. 7 Derivation 2); (ii)
   rank loss of $D(h,m)$ means $\nabla h \parallel \nabla m$; (iii) the ring's minimum ray is
   $-\nabla d_i$, so the test is one angle comparison. *Collapsible:* why raster scans miss ceiling
   critical points (C Fig. 6.22); the cycle algorithm's forward, reverse, and closing phases [C ref.
   11] that guarantee the closing critical point (C Fig. 6.23).
5. **Euler's count: $N_{ce} = N_{cp} + N_{ob} - 1$** (C §6.2.5). *Statement:* for a boustrophedon
   decomposition of a bounded planar free space, cells $=$ critical points $+$ obstacles $- 1$.
   *Sketch (4 steps):* (i) the Reeb graph is connected and planar with $v = N_{cp}$, $e = N_{ce}$;
   (ii) its faces are the obstacles plus the outer boundary, $f = N_{ob} + 1$; (iii) Euler: $v - e +
   f = 2$; (iv) rearrange. The outer boundary's leftmost and rightmost points count as critical
   points (slice birth and death). *Collapsible:* C Fig. 6.25's check ($21 + 2 - 1 = 22$); cells
   grow linearly with discovered critical points.
6. **Coverage path length is linear in area, critical points, and perimeter** (C §6.2.5).
   *Statement:* the sensor-based boustrophedon path is shorter than $\Delta^2/(2r) + 4\Delta N_{ce}
   + 5 P_{total} = \Delta^2/(2r) + 4\Delta(N_{cp} + N_{ob}) + 5P_{total} - 4\Delta$. *Sketch (5
   steps):* (i) laps: at most $\lceil \Delta/2r \rceil + N_{ce}$ of length $\le \Delta$ (one extra
   first lap per cell, C Fig. 6.26); (ii) boundary following: $1.5 P_{cell}$ to $2 P_{cell}$ per
   cell (undo-reverse, C Fig. 6.27), total $< 2P_{total}$; (iii) backtracking: $\le P_{cell} +
   \Delta$ each, total $\le P_{total} + \Delta N_{ce}$; (iv) discovery $\le P_{total}$, DFS
   traversal of covered cells $\le P_{total} + 2 N_{ce}\Delta$; (v) sum. *Collapsible:* each bound's
   figure (C Figs. 6.26–6.30); DFS visits each cell once [C ref. 118].
7. **Coda — pursuers needed** (C §6.3). *Statement:* a simply connected polygon with $n$ edges can
   be cleared by $O(\log n)$ pursuers; with $h$ holes, $O(\sqrt h + \log n)$. *Sketch (4 steps):*
   (i) splitting $\mathcal{W}_{free}$ by a chord held by one static pursuer gives $H \le k + 1$ from
   $H \le k$ on each part; (ii) a split keeping a third of the edges per side gives depth $O(\log
   n)$; (iii) with holes, remove the $O(h)$ trichromatic triangles and apply the planar separator
   theorem with $O(\sqrt h)$ static pursuers; (iv) recurse. *Collapsible:* conservative regions by
   rays from convex vertices (three cases, C Fig. 6.35); the four gap-edge transition rules; $G_i$
   and its all-zeros goal nodes (C Figs. 6.38–6.39); the U-shaped worst case (C Fig. 6.40).

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| Trapezoidal sweep (C §6.1) | `(polys) -> Decomposition` | $O(n \log n)$ |
| Point location (C §6.1) | `(decomp, q) -> cell` | $O(\log n)$ via slabs |
| Adjacency-graph planning (C Ch. 6 intro; Ch. 6 `astar`) | `(decomp, q_s, q_g) -> Path` | $O(E \log V)$ + midpoint–centroid chaining |
| Boustrophedon decomposition (C §6.2.1) | `(polys, h) -> MorseDecomposition` | $O(n \log n)$ sweep, events only at critical points |
| Exhaustive walk (C §6.2.1; Ch. 6 DFS) | `(adjacency) -> Vec<cell>` | $O(V + E)$ |
| Within-cell lapping (C §6.2.1, §6.2.2) | `(cell, 2r) -> Vec<Point2>` | $O(\text{laps})$ |
| Variable-slice decompositions (C §6.2.3) | `(world, h) -> MorseDecomposition` | sweep in $h$ |
| Cycle algorithm (C §6.2.4 [C ref. 11]) | `Coverer::tick(scan) -> Phase` | online; bound of Derivation 6 |
| Information-graph search (C §6.3) | `(conservative cells, B_0) -> Vec<cell>` | BFS on $G_i$ to an all-zeros node |

**Numeric micro-example** (the chapter's contract with the unit test). A polygonal Workbench in
general position: boundary $B = (0, 0.2), (10, 0), (10.4, 6), (0.3, 6.2)$; triangle $T = (2, 2), (4,
1.5), (3, 4)$; quadrilateral $K = (6, 1), (8, 0.8), (8.6, 4), (6.5, 4.4)$. The eleven vertex
$x$-coordinates $0, 0.3, 2, 3, 4, 6, 6.5, 8, 8.6, 10, 10.4$ are distinct. *Trapezoidal.* Sweep left
to right counting cells *opened*: $B$'s leftmost vertex opens 1; $(0.3, 6.2)$ (upper chain,
extension down) closes 1, opens 1; $T$'s leftmost $(2,2)$ closes 1, opens 2; $(3,4)$ (upper hull,
extension up) $+1$; $(4,1.5)$ (rightmost) closes 2, opens 1; $K$: $(6,1)$ $+2$, $(6.5,4.4)$ $+1$,
$(8,0.8)$ $+1$, $(8.6,4)$ $+1$; $(10,0)$ (lower chain, extension up) $+1$; $(10.4, 6)$ closes the
last cell. Total **12 cells**: $3$ from the boundary, $4$ from the triangle, $5$ from the
quadrilateral — in general $(m-1) + \sum_i (k_i + 1)$ for a convex $m$-gon boundary with convex
$k_i$-gon obstacles. Vertical extensions: $2$ (boundary pass-through vertices) $+ 5$ (triangle: $2 +
1 + 2$) $+ 6$ (quad: $2 + 1 + 1 + 2$) $=$ **13 adjacency edges**. Euler check: $12 - 13 + f = 2
\Rightarrow f = 3 =$ two obstacles plus the outside. *Boustrophedon.* Critical points are the
vertices with extensions both up and down, plus the slice's birth and death: $(0,0.2)$, $(2,2)$,
$(4,1.5)$, $(6,1)$, $(8.6,4)$, $(10.4,6)$ — $N_{cp} = 6$, $N_{ob} = 2$, so $N_{ce} = 6 + 2 - 1 =$
**7 cells** (sweep check: $1 \to 2 \to 1 \to 2 \to 1$; cells opened $1 + 2 + 1 + 2 + 1 = 7$). The
Reeb graph has 6 nodes and 7 edges. With a $2r = 0.5$ effector, the first cell ($x \in [0, 2]$)
needs $\lceil 2/0.5 \rceil = 4$ laps.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, carried end to end: **"mow the lawn, but let the hedges tell you where the lawn
changes shape."** Colors per the book code: obstacles slate, cell boundaries and Reeb graph in the
graph color, coverage path purple (covered area tinted likewise), Rusty orange with its range ring,
start green; in w10.2 contaminated regions are hatched amber (the C-obstacle role, reused as "unsafe
to assume clear") and the pursuer orange.

- **Widget w10.1: Boustrophedon Mower** — type: wasm-sim. The §3 Workbench (then an Apartment room);
  a vertical slice sweeps; at each critical point a cell boundary snaps in and the Reeb graph gains
  a node; then Rusty mows cell by cell along the walk. **Manipulates:** headline — effector width
  $2r$ (slider $0.2 \to 1.0$ m); disclosure: trapezoidal vs. boustrophedon, drag vertices (general
  position kept by a tiny jitter), slice function $h$, seed re-roll. **Observes:** cell count $12
  \to 7$ on switching; $N_{ce} = N_{cp} + N_{ob} - 1$ live; path length vs. the Derivation 6 bound;
  covered fraction reaching 100 %. **Misconception killed:** "more cells means finer coverage" —
  more cells means more turnarounds.
- **Widget w10.2: Pursuit-Evasion** — type: interactive animation. C Fig. 6.37's polygon and a
  U-shaped world; the dragged pursuer's $V(x)$, 0/1 gap-edge labels, and $B(x)$ are shown, with
  $G_i$ beside the world and the current node lit. **Manipulates:** headline — the pursuer's
  position (drag); disclosure: a second pursuer, "auto-solve" (search $G_i$ for an all-zeros node,
  replay), world picker. **Observes:** a gap-edge flipping to 1 when the pursuer retreats
  (recontamination); the four transition rules firing at cell crossings; in the U-world one pursuer
  fails forever, two succeed. **Misconception killed:** "seeing a region once clears it" —
  information, not geometry, is the state.
- **Widget w10.3: Trapezoid Sweep** — type: interactive animation. The §3 Workbench with the sweep
  line paused at each of the eleven events; $L$ shown as a strip of edge labels (C Figs. 6.5–6.8).
  **Manipulates:** headline — event index (scrub); disclosure: color-coded event types, adjacency
  overlay. **Observes:** $L$ gaining and losing edges; extensions only into free space; totals
  reaching 12 cells, 13 edges. **Misconception killed:** "every vertex gets two extensions" — only
  those with free space above and below do: the boustrophedon critical points.

Dashboard layout: w10.1 full-width atop the C section with the Reeb graph docked right; w10.3
half-width beside §6.1; w10.2 half-width in the coda. All autoplay seeded defaults and degrade to
static SVG.

## 5. Practical (P) — Rust Implementation

**Crates:** `petgraph` 0.8 (adjacency, Reeb, information graphs), `parry2d` 0.30 (`parry2d-f64`;
segment intersections via the Ch. 2 `collide` trait), `nalgebra` 0.35, `rand` 0.9 (`SmallRng`),
`search` (Ch. 6), `potential` (Ch. 7), `roadmap` (Ch. 8 GVD), `bugs` (Ch. 3).

**Module plan:**
- `crates/cells/` — **introduced here.** `src/lib.rs` (`Decomposition`, `Cell`, adjacency),
  `src/trapezoid.rs` (sweep, four event types), `src/locate.rs` (slab point location),
  `src/morse.rs` (`SliceFn`, `boustrophedon`, variable slices, critical-point detection),
  `src/reeb.rs`, `src/coverage.rs` (exhaustive walk, lapping, Derivation 6 accounting),
  `src/online.rs` (`Coverer`: the cycle algorithm), `src/pursuit.rs` (visibility polygons,
  conservative regions, information graph), `examples/{workbench_counts, mow_apartment,
  pursue_u}.rs`.
- `web/lib/cells/` — TypeScript port for w10.1–w10.3; `__checks__.ts` pins 12 / 13 / 7 / 6.

**Key types & signatures:**

```rust
pub struct PolyWorld { pub boundary: cspace::Polygon, pub obstacles: Vec<cspace::Polygon> }

pub struct Decomposition { pub cells: Vec<Cell>, pub adjacency: UnGraph<usize, Boundary> }
pub struct Cell { pub polygon: Vec<Point2<f64>>, pub x_range: (f64, f64) }
pub struct Boundary { pub segment: (Point2<f64>, Point2<f64>) }   // a vertical extension
// src/trapezoid.rs — C §6.1
pub enum EventKind { BothRight, BothLeft, LowerLeftUpperRight, LowerRightUpperLeft } // C Fig. 6.4
/// Sort vertices by x (general position asserted); sweep with `L` a BTreeMap keyed by edge y.
pub fn trapezoidal(world: &PolyWorld) -> Decomposition;
pub fn locate(d: &Decomposition, q: Point2<f64>) -> Option<usize>;
/// Midpoint–centroid chaining over a Ch. 6 `astar` result on the adjacency graph (Derivation 2).
pub fn plan(d: &Decomposition, q_s: Point2<f64>, q_g: Point2<f64>) -> Option<Vec<Point2<f64>>>;
// src/morse.rs — C §6.2
pub trait SliceFn { fn h(&self, q: Point2<f64>) -> f64; fn grad_h(&self, q: Point2<f64>) -> nalgebra::Vector2<f64>; }
pub struct Vertical; pub struct Radial { pub center: Point2<f64> } pub struct Squarel { pub center: Point2<f64> }
pub struct CriticalPoint { pub q: Point2<f64>, pub kind: Split /* 1→2 */ | Merge /* 2→1 */ | Birth | Death }
pub fn boustrophedon(world: &PolyWorld, h: &dyn SliceFn) -> (Decomposition, Vec<CriticalPoint>);
// src/reeb.rs — C §6.2.4
pub struct ReebGraph { pub graph: UnGraph<CriticalPoint, usize /* cell id */> }
impl ReebGraph { pub fn euler_check(&self, n_obstacles: usize) -> bool { /* e == v + n_ob - 1 */ } }
// src/coverage.rs
pub struct CoveragePath { pub points: Vec<Point2<f64>>, pub laps: usize, pub boundary_len: f64, pub backtrack_len: f64 }
/// DFS exhaustive walk (Ch. 6), then laps of spacing `2r` per cell; `bound()` = Derivation 6.
pub fn coverage_path(d: &Decomposition, world: &PolyWorld, two_r: f64) -> CoveragePath;
pub fn coverage_fraction(path: &CoveragePath, world: &PolyWorld, two_r: f64, raster: f64) -> f64;
// src/online.rs — C §6.2.4 cycle algorithm, sensor-based
pub enum Phase { Forward, Reverse, Closing, Backtrack, Done }
pub struct Coverer<'r> { pub robot: &'r mut sim::Rusty, pub reeb: ReebGraph, pub two_r: f64, pub phase: Phase }
impl Coverer<'_> {
        pub fn tick(&mut self, scan: &bugs::RangeScan, dt: f64) -> Phase;
}
// src/pursuit.rs — C §6.3 (coda)
pub struct InfoState { pub cell: usize, pub labels: Vec<u8> }
pub fn conservative_regions(world: &PolyWorld) -> Decomposition;   // rays from convex vertices, 3 cases
pub fn information_graph(d: &Decomposition, world: &PolyWorld) -> petgraph::graph::DiGraph<InfoState, ()>;
pub fn clear(world: &PolyWorld, start: Point2<f64>) -> Option<Vec<Point2<f64>>>; // BFS to an all-zeros node
```

**Worked end-to-end example:** `cargo run --example workbench_counts -p cells` builds the §3 world
and prints `trapezoidal: 12 cells, 13 adjacency edges, euler faces = 3`, then `boustrophedon: 6
critical points, 7 cells, reeb nodes = 6, reeb edges = 7, euler_check = true`, then `laps in cell 0
at 2r = 0.5: 4`; `#[test] fn workbench_counts_match_text()` pins all of these. `#[test] fn
euler_holds_on_random_worlds()` asserts $N_{ce} = N_{cp} + N_{ob} - 1$ and the closed form $(m-1) +
\sum (k_i + 1)$ on 200 seeded convex-obstacle worlds in general position. `cargo run --example
mow_apartment --seed 10` covers an Apartment room offline and online, printing `offline: length 41.3
m, covered 100.0 %` and `online: length 52.7 m, covered 100.0 %, bound 118.4 m` (lengths logged; the
test pins `covered == 100 %` at $5$ cm raster and `length <= bound`). `cargo run --example pursue_u`
prints pursuers for C Fig. 6.40's U-worlds (`12 edges: 1, 36 edges: 2`).

**Runnable artifact:** w10.1 and w10.3 are `web/lib/cells/` ports of `trapezoidal`, `boustrophedon`,
`coverage_path`, and `ReebGraph`; w10.2 ports `pursuit`. The Integration lab runs the offline/online
comparison in the Apartment (Rusty's range ring) and on the Workbench (Reach's end-effector
footprint), reporting length, covered fraction, and the bound.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w10.1 | Boustrophedon Mower | wasm-sim | `cells` → `web/lib/cells/`, `search` DFS, `bugs` follower, `sim` Rusty | $2r$ slider, decomposition toggle, drag vertices, slice function, seed re-roll | cells come from critical points; fewer cells, shorter path; Euler's count |
| w10.2 | Pursuit-Evasion | interactive animation | `cells::pursuit` → TS port | drag pursuer, second pursuer, auto-solve, world picker | information state, recontamination, $O(\log n)$ pursuers |
| w10.3 | Trapezoid Sweep | interactive animation | `cells::trapezoid` → TS port | event scrub, event-type colors, adjacency overlay | four event types; extensions only into free space; 12 cells / 13 edges |
| — | Missed-sofa hook | animation (autoplay) | `cells::online`, `sim` | none (replay) | raster coverage misses closing critical points |
| f10.4 | Slice-function gallery | static-svg | build-time SVG | hover | boustrophedon / spiral / spike / squarel / brushfire / wave-front cells (C Figs. 6.12–6.18) |

## 7. Exercises & Extensions

1. **(F)** Derive the trapezoidal cell count $(m-1) + \sum_i (k_i + 1)$ for a convex $m$-gon
   boundary with convex $k_i$-gon obstacles in general position; show by a notched obstacle that
   reflex vertices break it, and give the corrected per-vertex rule.
2. **(F)** Prove that a slice-sense critical point cannot lie inside a slice interval (C §6.2.2),
   and that for $h = |x| + |y|$ critical points occur where an obstacle's normal lies in the convex
   hull of the two flat sides meeting at a corner (C §6.2.3).
3. **(C, predict-then-verify)** In w10.1, drag the triangle's top vertex $(3,4)$ to $(3,1.8)$.
   Predict both cell counts before releasing; verify, and explain which vertex became a critical
   point.
4. **(C)** In w10.2's U-world with one pursuer, find the information-graph cycle that prevents
   clearing; add a second pursuer and read off which gap-edge it holds at 0.
5. **(P)** Implement `Radial` and `Squarel` critical-point detection for polygons (circle or diamond
   tangent to an edge or vertex); render C Figs. 6.12 and 6.14 for the §3 Workbench.
6. **(P, stretch)** Implement the brushfire decomposition (C §6.2.3) from Ch. 7's `brushfire` and
   Ch. 8's GVD (cells born at saddles of $D$, C Fig. 6.17's pattern); compare its length with the
   boustrophedon path.

## 8. Modernization Notes

- **Kept from Choset, deliberately:** the trapezoidal sweep with its four event types and list trace
  — the book's cleanest sweep-line introduction; Morse decompositions as the unifying definition
  (Def. 6.2.1) with the variable-slice gallery intact (one theorem, six patterns); sensor-based
  coverage with the cycle algorithm and the missed-critical-point failure; Euler's count and the
  path-length bound in full, the book's only quantitative coverage guarantees; the pursuit/evasion
  coda, kept short, because it carves cells by information, not geometry.
- **Added beyond 2005:** the structural identification of boustrophedon critical points with Ch. 9's
  Lemma 5.5.2 and the Ch. 8 Morse box; a reproducible cell-count micro-world with Euler checks as
  tests; coverage *measured* (covered fraction on a raster), not asserted; modern coverage pointers,
  each (verify): Galceran & Carpin's survey (2013); Choset's own coverage survey (2001); grid-based
  complete-coverage planners in field stacks (Nav2-style coverage servers); multi-robot
  boustrophedon partitioning (verify); the cross-link to frontier exploration in sister Ch. 24 —
  coverage of the *unknown* under a belief.
- **Dropped/condensed:** 3D generalizations (C Problems 3, 5) to exercises; the tethered-robot
  reading of the wave-front decomposition to a sentence; the full information-graph construction to
  the coda, LaValle & Guibas [C refs. 172, 273] as pointer; the Nomad experiment (C Fig. 6.21) to a
  caption.
- **Honesty items the prose must keep:** (i) general position is an assumption, enforced in code by
  a tiny deterministic jitter the text discloses; (ii) the path-length bound is loose (the lab
  prints bound and actual side by side); (iii) sensor-based critical-point detection needs a 360°
  range ring and fails on degenerate (axis-aligned) critical points [C ref. 86], which the
  Apartment's rectilinear walls produce — the text says how `online.rs` handles them (a merged-event
  rule) and that this is beyond Choset; (iv) pursuit/evasion is a workspace problem for point agents
  (C Problem 7), not a configuration-space one.
