# Chapter 6 — Graph Search for Planners

> Part II — Classical Planners: Search, Potentials, Roadmaps, Cells · Estimated length: 9 web pages · Difficulty: Foundational

## 1. Purpose & Story Arc

Every planner in Parts II–III ends the same way: with a graph and a question — *which vertices lead
from here to there, cheapest?* This chapter builds the one tool they all share. The reader knows
what $\mathcal{Q}_{free}$ is and how to test a configuration for collision; none of that tells
Rusty which way to drive. The "aha" is one sentence the chapter unpacks: **best-first search is one
algorithm with one knob.** Priority by edge count gives breadth-first search; by cost-to-come $g$,
Dijkstra; by a guess $h$ alone, greedy search; by $g + h$ with an *optimistic* $h$, A\* — optimal
and lazy at once. Push the knob past optimism and optimality quietly breaks; the chapter shows
exactly where. Two consequences follow: a search can be *repaired* rather than redone when the
world changes (D\*, D\* Lite), and a search run *backward from the goal* yields a plan for every
vertex at once — the reader's first value function, which the sister book turns into MDPs. Reader
path: Hook → C (play) → F (rigor) → P (Rust) → Integration lab → Exercises.

**Story line**
1. **Hook** — Rusty in the Apartment, goal in the next room; a "drive toward the goal" controller
   wedges into the first wall (autoplay, Ch. 2 `sim`). Something must *look ahead*.
2. **Play (C)** — w6.1 Search Theater: four searches expand on the same grid, the frontier painted
   in the graph color, the solution in the path color; the reader drags the heuristic-weight slider
   and watches A\* turn greedy and the path get longer.
3. **Formalize (F)** — graphs, grids as graphs, link length vs. path length; open/closed sets;
   admissible and consistent heuristics; the optimality proof; the non-optimistic counterexample
   worked by hand; running time, complexity classes, and the three completeness notions.
4. **Algorithm** — Choset's Alg. 24 (A\*) as the master template with BFS/DFS/greedy/Dijkstra as
   special cases; D\* (Choset Algs. 25–31) for local repair; D\* Lite and ARA\* as descendants;
   backward Dijkstra as a universal plan.
5. **Implementation (P)** — the `search` crate: one generic `best_first` over a `SearchSpace`
   trait; grids and `petgraph` graphs as two implementations; `pathfinding` as the cross-check.
6. **Integration lab** — the Apartment as an 8-connected lattice; a door closes while Rusty is en
   route and D\* Lite repairs only the cells whose costs actually changed.
7. **Close** — the universal-plan field over the whole Apartment: perturb Rusty anywhere, the
   arrows still bring it home. Forward pointers to Ch. 7 (wave-front *is* BFS) and sister Ch. 21.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (the three worlds, `sim` port of Rusty and the Apartment, `collide` for
  cell occupancy, widget framework), Ch. 4 (the grid rasterizer of $\mathcal{Q}_{free}$ for a disc
  robot — the lattice this chapter searches), Ch. 1 (the taxonomy, used here as vocabulary).
- **Feeds into:** Ch. 7 (wave-front and brushfire are BFS from a different seed set), Ch. 8 (A\* on
  the visibility graph and the GVD), Ch. 10 (search on a cell decomposition's adjacency graph),
  Ch. 16 (`replan_on_belief` wraps this crate), Ch. 18 (A\* over $(q, \dot q)$ lattices), Ch. 21
  (hybrid A\*, state lattices), Ch. 22 (learned heuristics in the same `astar`), Appendix D.
- **Baseline sources:** C App. H — §H.1 graphs, trees, DFS/BFS, grid connectivity, queue vs.
  stack, greedy search (b. 521–527); §H.2 A\* by example (Figs. H.11–H.13), §H.2.1–H.2.3 notation,
  completeness/optimality, greedy and Dijkstra, §H.2.4 A\* on a grid with the $1/1.4/10000$ metric,
  §H.2.5 the nonoptimistic example (b. 527–536); §H.3 D\* with the gate example and Algs. 25–31
  (b. 536–545); §H.4 universal plans (b. 546). C App. G — §G.1 asymptotic notation (Def. G.1.1),
  §G.2 complexity classes (Defs. G.2.5–G.2.12), §G.3 completeness (Defs. G.3.1–G.3.3) (b. 513–520).
  Sister book: Ch. 20 (`…/chapters/ch20-motion-planning`, its simplified A\*), Ch. 21
  (`…/chapters/ch21-mdps`, value iteration as the stochastic generalization of §H.4), Ch. 13
  (`…/chapters/ch13-occupancy-grids`, the lattice substrate). Modernization set: Koenig &
  Likhachev, D\* Lite (AAAI 2002; T-RO 2005) (verify); Likhachev, Gordon & Thrun, ARA\* (NIPS 2003)
  (verify). D\* itself is Choset's [C ref. 397] (Stentz) and therefore classical.

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; $G=(V,E)$, $g(v)$, $h(v)$, $f(v)$ are book-wide):

| Symbol | Meaning |
|---|---|
| $\mathrm{Star}(n)$ | the set of nodes adjacent to $n$ (Choset's name for the neighbor set) |
| $c(n_1, n_2)$ | cost (weight) of the edge $n_1 \to n_2$; $c \ge 0$ throughout |
| $O$, $C$ | open set (priority queue) and closed set (expanded nodes) |
| $b(n)$ | back pointer: the parent of $n$ on the best known path from $q_{start}$ |
| $\ell(\pi)$ | link length of a path — number of edges, weights ignored |
| $k(X)$, $t(X)$, $r(X,Y)$ | D\*: minimum-heuristic key, tag $\in\{NEW, OPEN, CLOSED\}$, measured edge cost |
| $rhs(s)$, $k_m$ | D\* Lite: one-step lookahead value, key modifier (verify) |
| $\epsilon \ge 1$ | heuristic inflation factor (ARA\*, weighted A\*) |
| $\pi^\star : V \to V$ | universal plan (policy): the successor to take from every vertex |
| $O(\cdot), o(\cdot), \Omega(\cdot), \Theta(\cdot)$ | asymptotic bounds (C Def. G.1.1) |

**Definitions** (each a display box):
- *Graph, path, cycle, tree* (C §H.1): $G=(V,E)$ with directed or undirected edges and
  non-negative weights; a path is a node sequence with consecutive nodes adjacent; a cycle returns
  to its first node; a tree is a connected directed acyclic graph with a unique root.
- *Grid-induced graph* (C §H.1, Fig. H.4): nodes are pixels; four-point connectivity joins N/S/E/W
  neighbors, eight-point adds the diagonals. Unit axial and $1.4$ diagonal steps form Choset's
  grid metric (Fig. H.16) — not Euclidean ($2.4 \ne \sqrt 5$).
- *Link length vs. path length* (C §H.1): BFS minimizes the number of edges; Dijkstra minimizes
  weighted cost; they coincide only for equal weights.
- *Expansion, open set, closed set* (C §H.2.1): expanding $n$ moves it from $O$ to $C$ and inserts
  or improves each $x \in \mathrm{Star}(n) \setminus C$.
- *Admissible (optimistic) heuristic* (C §H.2): $h(n) \le h^\star(n)$, the true cost of the
  cheapest path from $n$ to $q_{goal}$, for every $n$.
- *Consistent heuristic* (added; not in Choset): $h(n) \le c(n, x) + h(x)$ for every edge, and
  $h(q_{goal}) = 0$. Consistency implies admissibility.
- *Universal plan / policy* (C §H.4): a mapping from every node to an action, not just along one path.
- *Complete / resolution complete / probabilistically complete* (C Defs. G.3.1–G.3.3): finds a
  solution or reports none in finite time; the same for a fixed resolution step $\epsilon > 0$;
  probability of finding an existing solution $\to 1$ as running time $\to \infty$.
- *P, NP, PSPACE* (C Defs. G.2.5, G.2.7, G.2.11), $P \subseteq NP \subseteq PSPACE \subseteq
  EXPTIME$ (C §G.2) — stated for Ch. 9's use of Canny's bound; the language-theoretic scaffolding
  (Defs. G.2.1–G.2.4) moves to Appendix D.

**Derivations / theorems:**

1. **BFS is optimal in link length.** *Statement:* with a FIFO queue, a node's depth at first
   dequeue is its shortest link length from $q_{start}$ — on a grid, the Manhattan (4-connected) or
   chessboard (8-connected) shortest path (C §H.1–H.2). *Sketch (4 steps):* (i) the queue holds nodes of
   depth $d$ followed by nodes of depth $d+1$; (ii) a node first enqueued from a depth-$d$ parent has
   depth $d+1$; (iii) no shorter path can exist, else its parent would have been dequeued earlier;
   (iv) on a unit-weight grid, link length equals the grid metric. *Collapsible:* the queue
   invariant; why DFS (LIFO) gives *a* path but not a shortest one (Choset Figs. H.8–H.9).
2. **A\* terminates and is complete** (C §H.2.2). *Statement:* on a finite graph A\* either pops
   $q_{goal}$ or empties $O$; in the latter case no path exists. *Sketch (3 steps):* (i) each
   expansion moves a node to $C$ (permanently under consistency; otherwise reopenings are finite
   because acyclic paths are); (ii) $V$ is finite, so expansions are finite; (iii) while a path
   exists the frontier contains a node on it. *Collapsible:* Choset's acyclic-path count verbatim.
3. **A\* is optimal under admissibility** (formalizing C §H.2.2). *Statement:* if $h$ is
   admissible, the first time $q_{goal}$ is popped $g(q_{goal}) = c^\star$. *Sketch (5 steps):*
   (i) suppose the goal is popped with $g(q_{goal}) > c^\star$; (ii) on some optimal path, let $v$
   be the first node not yet expanded; it is in $O$ with $g(v) = g^\star(v)$;
   (iii) $f(v) = g^\star(v) + h(v) \le g^\star(v) + h^\star(v) = c^\star$;
   (iv) $f(q_{goal}) = g(q_{goal}) > c^\star \ge f(v)$; (v) the queue would have popped $v$ first —
   contradiction. *Collapsible:* Alg. 24 exits when the goal is *popped*, while the §H.2
   walk-through keeps expanding nodes with $f$ below the pushed goal's — the same rule, since a node
   is popped only when nothing in $O$ beats it.
4. **Consistency means no node is ever reopened** (added). *Statement:* with consistent $h$, $f$
   is non-decreasing along any expansion sequence, so a node's $g$ is optimal when it enters $C$.
   *Sketch (3 steps):* (i) $f(x) = g(n) + c(n,x) + h(x) \ge g(n) + h(n) = f(n)$ by consistency;
   (ii) popped $f$-values are therefore monotone; (iii) a later cheaper path to a closed node would
   imply a smaller $f$ popped later — impossible. *Collapsible:* octile and Euclidean $h$ are
   consistent; inflated $\epsilon h$ is not, but keeps $c \le \epsilon c^\star$ (ARA\*, verify).
5. **The non-optimistic counterexample** (C §H.2.5, Fig. H.20, our rendering with explicit
   numbers). *Statement:* edges $S \to A$: 4, $A \to G$: 4, $S \to B$: 3, $B \to G$: 2; $h(S)=2$,
   $h(A)=3$, $h(B)=10$, $h(G)=0$. A\* returns $S, A, G$ (cost 8); the optimum is $S, B, G$
   (cost 5). *Sketch (4 steps):* (i) expand $S$: $f(A) = 4 + 3 = 7$,
   $f(B) = 3 + 10 = 13$; (ii) expand $A$: $f(G) = 8$; (iii) $8 < 13$, so $G$ is popped before $B$;
   (iv) the culprit: $h(B) = 10 > h^\star(B) = 2$. *Collapsible:* where step (iii) of item 3
   fails; one bad estimate anywhere on an optimal path suffices.
6. **D\* repairs locally; backward Dijkstra is a universal plan** (C §H.3–H.4). *Statement:*
   Dijkstra from $q_{goal}$ labels every node with its cost-to-goal and a back pointer (a policy);
   when an edge cost changes, only nodes whose best path used that edge need new labels.
   *Sketch (5 steps):* (i) backward Dijkstra = forward Dijkstra on the reversed graph; (ii) its
   labels $h(X)$ satisfy the fixed-point equation $h(X) = \min_Y [c(X,Y) + h(Y)]$ — a deterministic
   Bellman equation; (iii) a cost increase at $(X_c, Y)$ raises $h$ only for nodes whose back
   pointer chain passes through it (Choset's *raised* states, $k < h$); (iv) a *lowered* neighbor
   whose chain avoided the change can offer a better $h$ ($h(4,1) + 1.4 = 7.6$ in Fig. H.26);
   (v) propagation stops when $k_{min} \ge h(X_c)$ (Alg. 25, line 20). *Collapsible:* Choset's
   PROCESS-STATE (Alg. 31) case analysis; D\* Lite's $rhs$ values and key modifier $k_m$ (verify);
   sister Ch. 21, where $\min$ becomes an expectation and the plan becomes a value function.

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| BFS / DFS (C §H.1) | `(G, s, goal) -> Option<Path>` | $O(V + E)$ |
| Greedy best-first (C §H.1, §H.2.3) | `(G, s, goal, h) -> Option<Path>` | $O((V+E)\log V)$, no optimality |
| Dijkstra (C §H.2.3, $f = g$) | `(G, s) -> (dist[], parent[])` | $O((V+E)\log V)$ binary heap |
| C Alg. 24 A\* | `(G, s, goal, h) -> Option<(Path, cost)>` | worst case Dijkstra; expansions shrink with $h$ |
| Weighted A\* / ARA\* (verify) | `(G, s, goal, h, eps_schedule) -> impl Iterator<Path>` | anytime; cost $\le \epsilon\, c^\star$ per iterate |
| C Algs. 25–31 D\* | `(L, S, G) -> Path` with `modify_cost` | initial Dijkstra + local repair |
| D\* Lite (verify) | `DStarLite::replan(&mut self, changes) -> Option<Path>` | repair proportional to affected nodes |
| Universal plan (C §H.4) | `(G, goal) -> (cost_to_go[], policy[])` | one backward Dijkstra |

**Numeric micro-example** (the chapter's contract with the unit test; C §H.2.4 style). An
8-connected $4 \times 3$ grid, cells $(col, row)$, $col \in \{1,\dots,4\}$, $row \in \{1,2,3\}$;
obstacle cells $(2,1)$, $(2,2)$ are deleted from the graph (not Choset's $10000$-cost nodes, see
§8); $q_{start} = (1,1)$, $q_{goal} = (4,1)$; axial step $1$, diagonal $1.4$; heuristic
$h = 1.4\min(\Delta x, \Delta y) + |\Delta x - \Delta y|$ (exact on an empty grid, hence admissible
and consistent). Ties in $f$ break toward smaller $h$, then lexicographically.

| pop | expanded | $g$ | $h$ | $f$ | open set after expansion ($f$: cell) |
|---|---|---|---|---|---|
| 1 | $(1,1)$ | 0 | 3.0 | 3.0 | 4.4: $(1,2)$ |
| 2 | $(1,2)$ | 1.0 | 3.4 | 4.4 | 5.2: $(2,3)$ · 5.8: $(1,3)$ |
| 3 | $(2,3)$ | 2.4 | 2.8 | 5.2 | 5.2: $(3,2)$ · 5.8: $(1,3)$, $(3,3)$ |
| 4 | $(3,2)$ | 3.8 | 1.4 | 5.2 | 5.2: $(4,1)$ · 5.8: $(1,3)$, $(3,1)$, $(3,3)$, $(4,2)$ · 7.2: $(4,3)$ |
| 5 | $(4,1)$ | 5.2 | 0 | 5.2 | goal popped — done |

Path by back pointers: $(1,1) \to (1,2) \to (2,3) \to (3,2) \to (4,1)$, cost $1 + 1.4 + 1.4 + 1.4
= 5.2$. **Five** expansions. Dijkstra ($h \equiv 0$) on the same grid expands all **ten** free cells
(every free cell has $g \le 5.2$ and the goal's $g$ is the largest) and returns the same cost 5.2.
Greedy ($f = h$) also finds this path here but is not guaranteed to. The non-optimistic
counterexample of derivation 5 (cost 8 returned, 5 optimal) is the second pinned number.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, carried end to end: **"one search, one knob."** Colors per the book code:
start green, goal red, obstacle slate, the explored set and frontier in the graph color (blue, two
tints: closed darker, open lighter), the solution path purple, Rusty orange.

- **Widget w6.1: Search Theater** — type: wasm-sim. Four lanes on one $24 \times 16$ 8-connected
  grid (a rasterized Apartment room): BFS, Dijkstra, greedy, A\*. Autoplays one synchronized run
  per load. **Manipulates:** headline — the heuristic weight $\epsilon$ in $f = g + \epsilon h$,
  slider $0 \to 3$ ($0$ turns A\* into Dijkstra, large $\epsilon$ into greedy); behind a
  disclosure: 4- vs. 8-connectivity, drag start/goal, paint obstacles, step/scrub, seed re-roll
  for the random obstacle layout. **Observes:** expansion count and path cost per lane; A\*'s cost
  equal to Dijkstra's for $\epsilon \le 1$ and drifting above it after; an "optimality lost" badge
  at the first $\epsilon$ where it happens. **Misconception killed:** "a better heuristic always
  means a better path" — a *stronger* heuristic means fewer expansions only while it stays
  optimistic; past that it buys speed with path quality.
- **Widget w6.2: D\* Replanner** — type: wasm-sim. Choset's gate scenario (§H.3) on a $7 \times 6$
  grid, then the Apartment. Rusty follows the planned path; at a chosen step a gate cell flips to
  obstacle. **Manipulates:** headline — the step at which the gate closes (scrub); disclosure: D\*
  vs. D\* Lite vs. from-scratch A\*, number of simultaneous changes, seed re-roll. **Observes:**
  cells touched by the repair (count shown) versus the full-grid flood of a from-scratch A\*;
  raised states (orange outline, $k < h$) and lowered states; the new path. **Misconception
  killed:** "replanning means searching again" — most of the old search is still correct, and D\*
  proves which part.
- **Widget w6.3: Universal Plan Field** — type: interactive animation. Backward Dijkstra from the
  goal fills the Apartment lattice with cost-to-go (heightfield colormap) and one arrow per cell.
  **Manipulates:** headline — drag or "kick" Rusty anywhere; the arrows take it home; disclosure:
  goal position, 4/8 connectivity. **Observes:** a policy defined everywhere; equal-cost contours
  that look like wave fronts (Ch. 7 preview); $h(X) = \min_Y [c(X,Y) + h(Y)]$ highlighted at the
  cell under the cursor. **Misconception killed:** "a plan is a path" — a plan can be a field, and
  that is what survives disturbance.

Dashboard layout: w6.1 full-width atop the C section (four lanes, shared scoreboard); w6.2
half-width beside the D\* derivation; w6.3 half-width at the close with a sticky equation panel.
Every widget autoplays a seeded default and degrades to a static SVG.

## 5. Practical (P) — Rust Implementation

**Crates:** `petgraph` 0.8 (`Graph`/`StableGraph` roadmaps as one `SearchSpace` impl),
`pathfinding` 4.15 (reference `astar`/`dijkstra`, tests only), `nalgebra` 0.35 (`Point2` cells),
`rand` 0.9 (`SmallRng` for seeded obstacle layouts). No search crate in the library path.

**Module plan:**
- `crates/search/` — **introduced by this chapter, imported by Chs. 7, 8, 10, 16, 18, 21, 22.**
  `src/space.rs` (`SearchSpace`, `Grid4`/`Grid8` over the Ch. 4 raster, `PetgraphSpace`),
  `src/best_first.rs` (one generic engine; `astar`/`dijkstra`/`bfs`/`greedy` wrappers),
  `src/dstar.rs` (Choset Algs. 25–31, the teachable version), `src/dstar_lite.rs` (default
  replanner), `src/ara.rs`, `src/policy.rs` (universal plan), `src/heuristics.rs`.
- `crates/search/examples/` — `astar_trace`, `nonoptimistic`, `dstar_gate`, `universal_plan`,
  `bench_table`. `web/lib/search/best_first.ts` — the TypeScript port powering w6.1–w6.3, pinned
  by `lib/__checks__.ts` to the micro-example numbers.

**Key types & signatures:**

```rust
// crates/search/src/space.rs
/// Anything best-first search can walk. Costs are non-negative edge weights
/// (Choset App. H.1); `Node` must be hashable so open/closed sets are maps.
pub trait SearchSpace {
    type Node: Copy + Eq + std::hash::Hash;
    /// Star(n) with edge costs c(n, x) — Choset's notation, made an iterator.
    fn neighbors(&self, n: Self::Node) -> impl Iterator<Item = (Self::Node, f64)> + '_;
}

/// 8-connected lattice over a rasterized Q_free (Ch. 4 `cspace::Raster`); `diag` is
/// 1.4 for Choset's grid metric (Fig. H.16) or sqrt(2) for the exact one.
pub struct Grid8<'a> { pub raster: &'a cspace::Raster, pub diag: f64 }

// crates/search/src/best_first.rs
/// The one knob: how a node's priority is formed from g and h.
pub enum Priority { LinkLength, CostToCome, HeuristicOnly, Sum { eps: f64 } }

pub struct Trace<N> { pub pops: Vec<(N, f64, f64)>, pub path: Vec<N>, pub cost: f64 }

/// BFS, Dijkstra, greedy, and (weighted) A* are this function with four `Priority` values.
/// Returns `None` when the open set empties — the completeness half of Derivation 2.
pub fn best_first<S: SearchSpace>(
    space: &S, start: S::Node, goal: S::Node,
    h: impl Fn(S::Node) -> f64, prio: Priority,
) -> Option<Trace<S::Node>>;

// crates/search/src/dstar_lite.rs  (`astar`/`dijkstra`/`bfs`/`greedy` are wrappers fixing `prio`)
/// Incremental replanner (Koenig & Likhachev, verify): g/rhs tables plus the key
/// modifier k_m, so the robot's own motion never forces a heap rebuild.
pub struct DStarLite<S: SearchSpace> { /* g, rhs, heap keyed (min(g,rhs)+h+k_m, min(g,rhs)) */ }
impl<S: SearchSpace> DStarLite<S> {
    pub fn new(space: S, start: S::Node, goal: S::Node) -> Self;
    /// Report observed cost changes; returns how many nodes' values moved (w6.2's statistic).
    pub fn update_edges(&mut self, changed: &[(S::Node, S::Node, f64)]) -> usize;
    pub fn next_step(&mut self) -> Option<S::Node>;
}

// crates/search/src/policy.rs
/// Choset §H.4: Dijkstra run backward from the goal yields cost-to-go and a successor
/// for *every* node — a universal plan. The sister book's Ch. 21 generalizes `min` to E[·].
pub struct UniversalPlan<N> { pub cost_to_go: HashMap<N, f64>, pub next: HashMap<N, N> }
pub fn universal_plan<S: SearchSpace>(space: &S, goal: S::Node) -> UniversalPlan<S::Node>;
```

**Worked end-to-end example:** `cargo run --example astar_trace -p search` builds the $4 \times 3$
grid of §3, prints the five-row pop table, the path `(1,1) (1,2) (2,3) (3,2) (4,1)`, `cost = 5.2`,
then `dijkstra: expansions = 10, cost = 5.2`. `#[test] fn astar_trace_matches_text()` asserts the
pop order, the $g/h/f$ triples (tolerance $10^{-9}$), the path, and both expansion counts.
`#[test] fn astar_equals_dijkstra_on_random_grids()` runs 200 seeded `SmallRng` grids and asserts
equal costs and equality with `pathfinding::prelude::astar`. `cargo run --example nonoptimistic`
prints `A* returned cost 8 via A; optimal is 5 via B` (`#[test] fn nonoptimistic_loses()`).
`cargo run --example dstar_gate` reproduces Choset's gate: initial path
$(2,1)\,(3,2)\,(4,3)\,(5,4)\,(6,5)\,(7,6)$, then the repaired $k(3,2) = 7.6$ via $(4,1)$
(C §H.3, Fig. H.26; `#[test] fn dstar_gate_repairs_to_7_6()`). `bench_table` prints expansions and
wall time for four searches on three Apartment queries (expansions regression-locked, time
informational).

**Runnable artifact:** w6.1 is `web/lib/search/best_first.ts`, the faithful port of `best_first`,
with `__checks__.ts` asserting the five-pop trace, cost 5.2, the ten Dijkstra expansions, and the
8-vs-5 counterexample; the crate's WASM build runs the same trace in CI as the reference. The
Integration lab rasterizes the Ch. 2 Apartment at 5 cm for disc-Rusty (Ch. 4 inflation), plans
with `astar`, drives Rusty with the Ch. 2 `sim`, closes a door cell at step 40, and lets
`DStarLite` repair — printing touched-node counts against a from-scratch `astar`.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w6.1 | Search Theater | wasm-sim | `search` → `web/lib/search/best_first.ts`, `SimCanvas`, seeded `SmallRng` layouts | $\epsilon$ slider, connectivity, drag start/goal, paint obstacles, scrub, seed re-roll | one search, one knob; where optimality breaks |
| w6.2 | D\* Replanner | wasm-sim | `search::dstar`, `search::dstar_lite` → TS port, `sim` for Rusty | gate-close step scrub, algorithm toggle, change count, seed re-roll | local repair vs. from-scratch; raised/lowered states |
| w6.3 | Universal Plan Field | interactive animation | `search::policy` → TS port, Apartment raster | drag/kick Rusty, move goal, connectivity | a plan can be a field; the deterministic Bellman equation |
| — | Goal-seeking controller hook | animation (autoplay) | `sim` | none (replay) | why look-ahead is needed |
| f6.4 | Complexity-class ladder | static-svg | build-time SVG | hover definitions | $P \subseteq NP \subseteq PSPACE \subseteq EXPTIME$; three completeness notions |

## 7. Exercises & Extensions

1. **(F)** Prove that the octile heuristic $1.4\min(\Delta x,\Delta y) + |\Delta x - \Delta y|$ is
   consistent on the 8-connected grid with Choset's $1/1.4$ metric, and show by a two-cell example
   that it is *not* admissible if diagonals cost $\sqrt 2$ instead of $1.4$.
2. **(F)** Choset's grid A\* (§H.2.4) keeps obstacle pixels as $10000$-cost nodes. Show that on
   any grid with fewer than $10000/1.4$ cells this returns the same path as deleting them, and
   exhibit a grid where the two differ.
3. **(C, predict-then-verify)** In w6.1 set $\epsilon = 2$. Before pressing Run, predict whether
   the A\* lane's cost can exceed Dijkstra's by more than a factor of 2. Run with ten seeds, then
   reconcile what you see with the weighted-A\* bound $c \le \epsilon\, c^\star$.
4. **(C)** In w6.2, find a gate position for which D\* Lite touches *more* nodes than a
   from-scratch A\* expands; explain via the raised/lowered classification why this is rare.
5. **(P)** Implement `Priority::LinkLength` as a true FIFO (not a heap with constant keys) and add
   a property test that BFS and Dijkstra agree on every unit-weight grid of 200 seeded trials.
6. **(P, stretch)** Implement ARA\* in `src/ara.rs` with schedule $\epsilon = 3, 2, 1.5, 1$,
   reusing the open set between iterations; add a fifth w6.1 lane; assert the final cost equals Dijkstra's.

## 8. Modernization Notes

- **Kept from Choset, deliberately:** App. H's by-example pedagogy (queue traces first, Alg. 24
  second); explicit open/closed bookkeeping; the $1/1.4$ grid metric, which makes hand arithmetic
  possible; the full D\* (Algs. 25–31), whose raised/lowered-state reasoning is the clearest
  explanation of *why* incremental search works; App. G's three completeness definitions, which
  the rest of the book uses as law; §H.4's universal plans, promoted to a widget as the bridge to
  value functions.
- **Added beyond 2005:** consistency and the no-reopening theorem (standard, absent from Choset);
  weighted A\*/ARA\* and the $\epsilon$-suboptimality bound (Likhachev, Gordon & Thrun 2003,
  verify); D\* Lite as the implementation default (Koenig & Likhachev 2002/2005, verify) with
  Choset's D\* as the teachable derivation; the §H.4 → sister Ch. 21 cross-link, stating once that
  backward Dijkstra is value iteration with deterministic transitions; a reproducible benchmark.
- **Dropped/condensed:** App. G's language-and-encoding formalism (Defs. G.2.1–G.2.4) goes to
  Appendix D; obstacle pixels as $10000$-cost nodes are replaced by deletion from the graph (what
  every modern grid planner does), kept as Exercise 2; the DFS link-length map (Figs. H.8–H.9)
  becomes one sentence.
- **Honesty items the prose must keep:** (i) Choset's D\* uses $h(X)$ for a cost-to-goal estimate,
  not A\*'s heuristic — the text names the clash and the code says `cost_to_go`; (ii) Alg. 24 as
  printed updates an improved open node's back pointer but not, explicitly, its $g$ and priority —
  ours does; (iii) the §H.2 walk-through and Alg. 24 differ in *when* they stop; Derivation 3
  reconciles them; (iv) benchmark times are informational, expansion counts are pinned.
