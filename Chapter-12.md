# Chapter 12 — Tree Planners: EST, RRT, and Friends

> Part III — Sampling-Based Planning · Estimated length: 9 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Chapter 11 spent its budget learning *all* of $\mathcal{Q}_{free}$ so that any query is cheap. Most
robots do not have many queries; they have one, now. This chapter inverts the economics: grow a
tree from the configuration you are in toward the one you want, sample *conditionally* on what the
tree already knows, and stop the instant the two meet. Two families do it with opposite instincts.
The Expansive-Spaces Tree picks a lonely node and pushes outward from it (forward propagation); the
Rapidly-exploring Random Tree picks a random target and pulls the nearest node toward it
(steering). The "aha": **RRT's pull is a Voronoi bias — the probability a node is chosen to grow
equals the volume of its Voronoi region, so the frontier grows fastest exactly where the tree has
not been** — one geometric fact that explains why a thirty-line algorithm covers spaces nobody can
visualize. Along the way the chapter adds `extend` to Chapter 11's `Steer`, imports its samplers
and kd-tree unchanged, and shows through Choset's SRT that PRM, RRT and EST are three settings of
one dial.

Story line:
1. **Hook:** the Ch. 11 roadmap for Reach took 1.8 s to build and answers a query in 4 ms. Move
   one obstacle: the roadmap is stale. An RRT answers the single query in 60 ms from scratch
   (autoplay, side by side).
2. **Play (C):** w12.1 Tree Grower on the torus; toggle the Voronoi overlay and watch the big cells
   get chosen.
3. **Formalize (F):** conditional sampling; EST's $\pi_T\propto1/w_T$; RRT's `extend` (C Alg. 11)
   and the Voronoi-bias lemma; probabilistic completeness for both families.
4. **Algorithms:** C Algs. 8–13; step size; goal bias $p$; RRT-Connect (classical); SBL (lazy
   checks, grid-cell selection); SRT (C §7.3, Alg. 14) and the parameter table that collapses it
   to PRM, RRT or EST; KPIECE (modern, verify) as the projection-based descendant of EST's grid.
5. **Implementation (P):** `sampling::tree` — one `Tree<M>`, `Est`, `Rrt`, `RrtConnect`, `Sbl`,
   `Srt`; three hand-traced extensions pinned by a test.
6. **Integration lab:** five planners on three worlds — including Hitch in the Lot with a
   straight-line steer that ignores the car's constraint, the deliberate wrong answer Chapter 14's
   preview and Chapter 21 fix; close on the question trees cannot answer: how good is the path?

## 2. Prerequisites & Position

- **Builds on:** Ch. 11 (**everything imported, nothing re-introduced**: `Sampler` + `Uniform`,
  `NearestNeighbors` + `KdTree`, `Steer` + `StraightLine`, `FreeSpace`, the
  $(\epsilon,\alpha,\beta)$ vocabulary, Theorem 7.4.1's tiling), Ch. 5 (`Manifold::interpolate` is
  what `extend` moves along), Ch. 6 (parent-following path extraction; the lattice A\* lane),
  Ch. 7 (RPP's randomized-potential ancestry; why pure goal bias is a potential planner), Ch. 2
  (worlds; Hitch's kinematics exist but are deliberately *not* used by the steer here).
- **Feeds into:** Ch. 13 (RRT\*/RRG/Informed RRT\*/BIT\* are this `Tree<M>` plus costs and a radius
  neighbor set; "RRT never improves" is about *this* RRT), Ch. 14 (the forward-propagation preview
  replaces `extend` with a simulator — EST's instinct returns; §7.5.2's incremental multi-robot
  extension), Ch. 21 (kinodynamic RRT in full; Dubins `Steer`), Ch. 23 (Reach's single-query
  planner).
- **Baseline sources:** C §7.2 intro (b. 227–229: RPP, Ariadne's clew, PRM as single-query, Lazy
  PRM), §7.2.1 (b. 230–233: EST, Algs. 8–9, $\pi_T$, merging), §7.2.2 (b. 233–238: RRT, Algs.
  10–13, step size, metric sensitivity, goal bias, merging), §7.2.3 (b. 238: SBL), §7.3
  (b. 238–242: SRT, Alg. 14, parameters, parallel SRT). Sister book: Ch. 20
  (`…/chapters/ch20-motion-planning`; its RRT is the preview, this is the reference), Ch. 8
  (`…/chapters/ch08-nonparametric-filters`: EST's $\pi_T$ is a resampling weight), Ch. 23
  (`…/chapters/ch23-mppi`: rollouts as forward propagation, the bridge to Ch. 14). Modernization
  set: Şucan & Kavraki, KPIECE (WAFR 2008) (verify). RRT-Connect is *classical* [C ref. 249].

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped):

| Symbol | Meaning |
|---|---|
| $T=(V,E)$, $T_{init}$, $T_{goal}$, $\mathrm{par}(v)$ | tree rooted at $q_{start}$ / $q_{goal}$; parent map |
| $q_{rand}$, $q_{near}$, $q_{new}$ | sampled target; nearest tree node; node produced by `extend` |
| $\eta$ | `step_size`: how far `extend` moves from $q_{near}$ toward $q_{rand}$ |
| $p$ | goal bias: probability that $q_{rand}:=q_{goal}$ |
| $\pi_T(q)$, $w_T(q)$ | EST selection distribution over $V$; density weight (nodes within a neighborhood of $q$) |
| $\mathrm{Vor}_T(v)$ | Voronoi region of $v$ with respect to $V$ under `dist` |
| $G_T=(\mathcal{V}_T,\mathcal{E}_T)$, $\ell$ | SRT roadmap of trees; merge attempts (C Alg. 13) |

**Definitions:**
- *Single-query planner.* Solves one pair; explores only the relevant part of $\mathcal{Q}_{free}$;
  every node is reachable from a root by construction.
- *Conditional sampling* (C §7.2). The next configuration's distribution depends on the current
  tree(s) and the query — unlike PRM's unconditional rain.
- *`extend` status.* `Reached` ($q_{new}=q_{rand}$), `Advanced` ($q_{new}$ added, $\eta$ short of
  $q_{rand}$), `Trapped` (node or local path blocked) — the three-valued return RRT-Connect's greedy
  loop needs.
- *Voronoi bias.* $\Pr[q_{near}=v]=\mu(\mathrm{Vor}_T(v)\cap\mathcal{Q}_{free})/\mu(\mathcal{Q}_{free})$
  under uniform $q_{rand}$.
- *Lazy collision checking* (SBL). Nodes checked on creation; an edge checked once, only when it
  lies on a candidate path joining the trees.
- *Roadmap of trees* (SRT). A PRM roadmap with each tree contracted to a vertex;
  $(T_i,T_j)\in\mathcal{E}_T$ iff some $q_i\in T_i$, $q_j\in T_j$ were joined by a local path.

**Derivations / theorems:**

1. **Voronoi-bias lemma for RRT.** *Statement:* with $q_{rand}\sim$ Uniform($\mathcal{Q}_{free}$)
   and `dist` a metric, node $v$ is extended with probability
   $\mu(\mathrm{Vor}_T(v)\cap\mathcal{Q}_{free})/\mu(\mathcal{Q}_{free})$; frontier nodes own the
   unexplored volume, so expected growth points outward. *Sketch (4 steps):* (i) $q_{near}$ is by
   definition the Voronoi owner of $q_{rand}$; (ii) uniform sampling turns volume into probability;
   (iii) interior cells shrink to the inter-node spacing as a region fills, frontier cells remain
   bounded only by $\mathcal{Q}$; (iv) hence the fraction of extensions spent inside decays as the
   tree densifies. *Collapsible:* LaValle & Kuffner [C ref. 272]; metric sensitivity — a badly
   scaled $SE(2)$ weighting distorts the cells (C §7.2.2; [C ref. 103, Cheng & LaValle]).
2. **Probabilistic completeness of RRT.** *Statement:* for a query with a path of clearance $\rho>0$
   and $\eta\le\rho$, the probability that the tree has not reached $B_{\rho/2}(q_{goal})$ after $n$
   iterations decays exponentially in $n$, with constants depending on $\rho$, $L$,
   $\mu(\mathcal{Q}_{free})$. *Sketch (5 steps):* (i) tile the path with $\lceil2L/\rho\rceil$ balls
   as in Ch. 11's Theorem 7.4.1; (ii) once the tree has a node in ball $i$, a sample in a suitable
   sub-ball of ball $i+1$ makes *some* node's extension land in ball $i+1$ — the delicate step, since
   $q_{near}$ need not be the node in ball $i$; (iii) lower-bound that event's probability per
   iteration by a constant $c>0$; (iv) advancing one ball is dominated by a geometric variable;
   (v) sum over balls and apply Chernoff. *Collapsible:* Choset points to [C ref. 271, LaValle &
   Kuffner]; the step-(ii) subtlety is why the rigorous modern proof (Kleinbort, Solovey,
   Littlefield, Bekris & Halperin, 2019) (verify) is cited — the 2005 text has no argument.
3. **EST covers an expansive space.** *Statement:* with $\pi_T\propto1/w_T$ in an
   $(\epsilon,\alpha,\beta)$-expansive space, $\mathrm{reach}(V)$ reaches $\ge\tfrac34$ of its
   component after $O(\beta^{-1}\ln4)$ lookout hits, each with per-attempt probability bounded below
   in terms of $\alpha\epsilon$; hence EST is probabilistically complete. *Sketch (4 steps):* (i) a
   low-$w_T$ node sits on the frontier; (ii) by expansiveness a fraction $\ge\alpha$ of the current
   reach set is lookout; (iii) a sample in the lookout extends the reach set by $\ge\beta$ of the
   remainder — one step of Ch. 11's linking sequence; (iv) Lemma 7.4.7 bounds the step count.
   *Collapsible:* Hsu's analysis [C ref. 192; 196, Hsu, Latombe & Motwani]; why uniform $\pi_T$
   oversamples the roots; the grid-cell $\pi_T$ (C §7.2.3) as an $O(1)$ substitute for the $O(n)$
   neighbor count.
4. **Goal bias: why $p=1$ is a potential-field planner.** *Statement:* with $q_{rand}\equiv
   q_{goal}$, extending the nearest node toward the goal is greedy descent of
   $\mathrm{dist}(\cdot,q_{goal})$ over the frontier and inherits Ch. 7's local minima; any $p<1$
   restores completeness, and small $p$ (Choset's experimental 0.05) speeds convergence. *Sketch (3
   steps):* (i) at $p=1$ the goal-nearest node is extended every iteration until `Trapped`;
   (ii) once trapped, no other node is ever chosen; (iii) for $p<1$ the uniform component gets a
   positive fraction of iterations, so derivation F.2 applies with constants scaled by $1-p$.
   *Collapsible:* sampling a region around $q_{goal}$ defined by the tree's closest nodes; RPP
   ancestry [C ref. 40].
5. **SRT specializes to PRM, RRT, and EST.** *Statement:* with one node per tree, one close pair
   per candidate edge and zero merge iterations, C Alg. 14 is C Alg. 6; with zero roadmap trees and
   zero close pairs it is a single RRT or EST. *Sketch (3 steps):* (i) a one-node tree's
   representative is the node, so tree neighbors are configuration neighbors; (ii) the one close
   pair checked by $\Delta$ is PRM's edge test; (iii) with no roadmap trees only $T_{init},T_{goal}$
   exist and the connection phase is the tree planner's merge. *Collapsible:* why SRT parallelizes (trees
   independent, edges not) [C ref. 14, Akinc et al.; 43, Bekris et al.].

**Named algorithms:**

| Algorithm | Signature | Complexity (per iteration unless noted) |
|---|---|---|
| C Alg. 8 / 9 BuildEST / ExtendEST | `(q0, n) -> T`; `(T, q) -> Option<q_new>` | 1 neighborhood sample + 1 call to $\Delta$; $\pi_T$ update $O(1)$ with cells, $O(n)$ naive |
| C Alg. 10 / 11 BuildRRT / ExtendRRT | `(q0, n) -> T`; `(T, q) -> Reached \| Advanced \| Trapped` | 1 NN query ($O(\log n)$ expected) + $\lceil\eta/\text{step}\rceil$ checks |
| C Alg. 12 ConnectRRT | `(T, q) -> connected \| failure` | $\le\lceil\mathrm{dist}/\eta\rceil$ extensions |
| C Alg. 13 MergeRRT | `(T1, T2, ℓ) -> merged \| failure` | $\le\ell$ extension pairs with tree swap |
| RRT-Connect (classical, [C ref. 249]) | `(q_start, q_goal, n) -> Option<Path>` | Alg. 13 with Alg. 12 on the second tree |
| SBL [C ref. 367, Sánchez & Latombe] | `(q_start, q_goal, n, grid) -> Option<Path>` | EST steps; checks only on candidate bridging paths |
| C Alg. 14 ConnectSRT | `(V_T, k, r) -> G_T` | per tree: $k+r$ neighbors × (close pairs + optional merge) |
| KPIECE (modern, verify) | `(q_start, goal_region, projection, n) -> Option<Path>` | cell importance over a multi-level projection grid |

**Numeric micro-example** (the contract with the unit test in §5). Point robot in
$\mathcal{Q}=[0,10]^2$, obstacle $\mathcal{QO}=[3,4]\times[0,3]$, root $q_0=(1,1)$, $\eta=1.0$,
Euclidean `dist`, straight-line `extend` with subdivision at $0.25$. The sample list is fixed and
replayed through Ch. 11's `Scripted` sampler, so the trace is independent of any RNG's bit stream
(the seeded `SmallRng` run is pinned by a separate snapshot test): $q^{(1)}_{rand}=(5,4)$,
$q^{(2)}_{rand}=(2,6)$, $q^{(3)}_{rand}=(4,1.5)$.
1. **Extension 1.** Nearest: $q_0$, $\mathrm{dist}=5$. Direction $(0.8,0.6)$; $q_{new}=(1.8,1.6)$;
   edge free (never enters $x\ge3$). `Advanced`.
2. **Extension 2.** Distances to $(2,6)$: $\sqrt{26}=5.099$ from $q_0$, $4.4045$ from $(1.8,1.6)$
   → $q_{near}=(1.8,1.6)$. Direction $(0.2,4.4)/4.4045$; $q_{new}=(1.8454,2.5990)$. `Advanced`.
3. **Extension 3.** Distances to $(4,1.5)$: $3.0414$, $2.2023$, $2.4187$ → $q_{near}=(1.8,1.6)$
   again (it becomes the branch point). Direction $(2.2,-0.1)/2.2023$; $q_{new}=(2.7990,1.5546)$,
   $x<3$ so node and edge are free. `Advanced`. With $\eta=1.5$ instead, $q_{new}=(3.2985,1.5319)$
   lies inside $\mathcal{QO}$ → `Trapped`, tree unchanged — the test checks this branch too.
Final tree: 4 nodes; parents $(1.8,1.6)\mapsto q_0$, $(1.8454,2.5990)\mapsto(1.8,1.6)$,
$(2.7990,1.5546)\mapsto(1.8,1.6)$; total edge length $3.000$.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"push from the lonely node, or pull toward the empty space"** — EST pushes,
RRT pulls, both ask the tree where it has not been. Colors: tree edges **blue** (second tint for
$T_{goal}$), start **green**, goal **red**, obstacle **slate**, C-obstacle **amber hatched** on the
torus, solution **purple**, Reach **orange** when executing.

- **Widget w12.1: Tree Grower** — type: wasm-sim (flagship). RRT in Reach's unwrapped torus on the
  Workbench; seam-crossing edges drawn through the seam. Autoplays to first solution, seed shown.
  **Manipulates:** headline — step size $\eta$ (small: dense short-edged trees; large: `Trapped`
  flashes at the amber band); disclosure — goal bias $p$ (at $p=1$ a "stuck: potential-field mode"
  badge appears, per F.4), EST/RRT switch (EST shows $w_T$ as node halo size), Voronoi overlay
  (cells shaded by area so the frontier glows), seed. **Observes:** iterations to first solution;
  nodes vs. iterations (the gap is `Trapped`); the largest Voronoi cells being chosen frame after
  frame. **Misconception killed:** *"RRT explores randomly"* — it explores where the Voronoi cells
  are large, a deterministic function of the tree.
- **Widget w12.2: Bidirectional Connect** — type: wasm-sim. Two trees for Rusty-as-disc in the
  Apartment reaching across a doorway. **Manipulates:** headline — the greediness dial:
  `Extend`/`Extend` (C Alg. 13 as written), `Extend`/`Connect` (RRT-Connect), `Connect`/`Connect`;
  disclosure — SBL mode (lazy edges dashed until checked; the check counter ticks only on the
  bridging path), $\ell$, seed. **Observes:** the merge frame, collision checks per planner on the
  same seed, path length before and after Ch. 11's `shortcut_greedy`. **Misconception killed:**
  *"greedier is always faster"* — `Connect`/`Connect` wins in open rooms and loses in clutter where
  long greedy runs end `Trapped`.
- **Widget w12.3: Roadmap-of-Trees Dial** — type: interactive animation. SRT on the Apartment.
  **Manipulates:** headline — nodes per tree (1 … 200); disclosure — close pairs per edge, merge
  iterations. **Observes:** at (1, 1, 0) the picture becomes w11.1's roadmap; with no roadmap trees
  it becomes w12.1's single tree; between, blobs joined by dashed merge edges (Choset Fig. 7.16).
  **Misconception killed:** *"PRM and RRT are different algorithms"* — two corners of one box.

Dashboard layout: w12.1 full-width at the top of C with the iterations/nodes sparkline beneath;
w12.2 half-width beside the RRT-Connect/SBL subsection; w12.3 half-width inside the §7.3
discussion; the lab's benchmark is a Nivo grouped bar chart (planner × world, medians, 50 seeds).

## 5. Practical (P) — Rust Implementation

**Crates:** everything from Ch. 11's `sampling` — no new planner dependency; `nalgebra` 0.35;
`rand` 0.9 `SmallRng`; `petgraph` 0.8 for SRT's roadmap-of-trees graph; `pathfinding` 4.15 for the
hook's lattice A\* lane. Hitch's Ch. 2 kinematics are linked but unused by the steer; the lab says so.

**Module plan:** `crates/sampling/src/tree/` (a submodule of the Ch. 11 crate, per TOC): `mod.rs`
(`Tree<M>`: node arena with parent indices and a Ch. 11 `KdTree<M>` kept in sync; `path_to_root`);
`extend.rs` (`ExtendStatus`, `trait Extend<M>: Steer<M>` with a blanket impl — the "Ch. 12 adds
`extend`" artifact); `est.rs` (`Est`, `Density::{Naive, GridCells}`); `rrt.rs` (`Rrt`, `GoalBias`,
`connect`, `merge`, `RrtConnect`); `sbl.rs` (`Sbl`: two `Est`s, grid-cell selection, a lazy edge
list checked only on bridging paths); `srt.rs` (`Srt` over `petgraph::UnGraph<TreeId, ()>` with
`nodes_per_tree`, `close_pairs`, `merge_iters`, `k`, `r`, and constructors `prm_like()` /
`rrt_like()` for derivation F.5); `kpiece.rs` (`Projection` trait; modern, verify);
`examples/three_extensions.rs`, `examples/tree_bench.rs`.

**Key types & signatures** (compiles in spirit; Ch. 11 items are imported, never redefined):

```rust
use manifold::Manifold;                                                   // Ch. 5
use sampling::{FreeSpace, Sampler, Steer, Chart, Uniform, nn::KdTree};              // Ch. 11
use rand::rngs::SmallRng;

pub type NodeId = usize;
pub struct Node<M: Manifold> { pub q: M::Point, pub parent: Option<NodeId>, pub depth: u32 }

/// One tree type for every planner in Part III. Ch. 13 wraps it with costs; it does not fork it.
pub struct Tree<M: Chart> { pub nodes: Vec<Node<M>>, nn: KdTree<M>, pub space: M }
impl<M: Chart> Tree<M> {
    pub fn rooted(space: M, q0: M::Point) -> Self;
    pub fn nearest(&self, q: &M::Point) -> (NodeId, f64);              // O(log n) expected
    pub fn add(&mut self, q: M::Point, parent: NodeId) -> NodeId;
    pub fn path_to_root(&self, v: NodeId) -> Vec<M::Point>;
}

/// The three-valued answer RRT-Connect's greedy loop needs (C Algs. 11–12).
pub enum ExtendStatus<P> { Reached(P), Advanced(P), Trapped }

/// Ch. 12's addition to Ch. 11's `Steer`: move at most `eta` toward `toward` along the
/// manifold's interpolant, then run the local planner on that short piece.
pub trait Extend<M: Manifold>: Steer<M> {
    fn extend(&self, space: &M, free: &dyn FreeSpace<M>, from: &M::Point, toward: &M::Point, eta: f64)
        -> ExtendStatus<M::Point> {
        let d = space.dist(from, toward);
        if d <= eta {                                   // close enough: try to reach exactly
            return match self.steer(space, free, from, toward) {
                Some(_) => ExtendStatus::Reached(toward.clone()), None => ExtendStatus::Trapped };
        }
        let q_new = space.interpolate(from, toward, eta / d);  // why: fixed stride = Voronoi pull
        match self.steer(space, free, from, &q_new) {
            Some(_) => ExtendStatus::Advanced(q_new), None => ExtendStatus::Trapped }
    }
}
impl<M: Manifold, S: Steer<M>> Extend<M> for S {}       // every Steer (incl. Ch. 21 Dubins) gets it

pub struct GoalBias { pub p: f64 }                       // default 0.05 (C §7.2.2)
pub struct Rrt<M: Chart, S: Extend<M>, Smp: Sampler<M>> {
    pub tree: Tree<M>, pub steer: S, pub sampler: Smp, pub eta: f64, pub bias: GoalBias,
    pub rng: SmallRng, pub trapped: u64,                 // counter the widgets display
}
impl<M: Chart, S: Extend<M>, Smp: Sampler<M>> Rrt<M, S, Smp> {
    /// C Alg. 11 driven by the sampler; returns the status so callers can count `Trapped`.
    pub fn extend_once(&mut self, free: &dyn FreeSpace<M>, goal: &M::Point) -> ExtendStatus<M::Point>;
    /// C Alg. 12: repeat extend toward `q` until Reached or Trapped.
    pub fn connect(&mut self, free: &dyn FreeSpace<M>, q: &M::Point) -> bool;
    /// C Alg. 10 + goal test, single tree.
    pub fn plan(&mut self, free: &dyn FreeSpace<M>, goal: &M::Point, n: usize) -> Option<Vec<M::Point>>;
}

/// C Alg. 13 with a greediness switch per side; RRT-Connect = (Extend, Connect) [C ref. 249].
pub enum Greed { Extend, Connect }
pub struct RrtConnect<M: Chart, S: Extend<M>> { pub a: Rrt<M, S, Uniform>, pub b: Rrt<M, S, Uniform>, pub greed: (Greed, Greed) }
impl<M: Chart, S: Extend<M>> RrtConnect<M, S> {
    pub fn merge(&mut self, free: &dyn FreeSpace<M>, attempts: usize) -> Option<Vec<M::Point>>;
}

```

**Worked end-to-end example:** `cargo run --example three_extensions -p sampling` builds the §3
world (`cspace::R2`, box obstacle), an `Rrt` with `StraightLine` (`Subdivision { step: 0.25 }`),
`eta = 1.0`, `GoalBias { p: 0.0 }`, and `Scripted` replaying $(5,4),(2,6),(4,1.5)$; prints each
iteration's `q_near`, distance, `q_new`, status and the final parent map — `(1.8, 1.6)`,
`(1.8454, 2.5990)`, `(2.7990, 1.5546)`, all `Advanced`, total length `3.000` — then reruns with
`eta = 1.5` and prints the `Trapped` third extension. `#[test] fn reproduces_three_extensions()`
asserts coordinates to $10^{-4}$ and statuses exactly; `#[test] fn seeded_rrt_snapshot()` pins the
first 20 nodes of a `SmallRng::seed_from_u64(12)` run for the TypeScript port;
`#[test] fn srt_degenerates_to_prm()` checks derivation F.5 against Ch. 11's six-sample edge set.
`tree_bench` runs the five planners on three worlds over 50 seeds and prints medians (iterations,
checks, path length) — the lab's table, pinned by a tolerance test.

**Runnable artifact:** the widgets run `web/lib/plan/{tree,rrt,est,sbl,srt}.ts`;
`lib/__checks__.ts` replays the three extensions and the seed-12 snapshot. The Integration lab is
the five-planner benchmark across Apartment, Workbench and Lot; its last row — Hitch with a
straight-line steer — yields a path the car cannot drive, drawn in orange with the sentence "a
steer must respect the robot; Chapter 14 previews the alternative, Chapter 21 does it properly."

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w12.1 | Tree Grower | wasm-sim | `sampling::tree` → `web/lib/plan/{tree,rrt,est}.ts` + `cspace` torus rasterizer + SimCanvas | $\eta$, goal bias, EST/RRT, Voronoi overlay, seed | Voronoi bias; step-size trade-off; $p=1$ is a potential field |
| w12.2 | Bidirectional Connect | wasm-sim | `web/lib/plan/{rrt,sbl}.ts` + `sim` Apartment | greediness dial, SBL toggle, $\ell$, seed | Extend vs. Connect; lazy checking's savings |
| w12.3 | Roadmap-of-Trees Dial | interactive animation | `web/lib/plan/srt.ts` | nodes/tree, close pairs, merge iterations | PRM, RRT, EST as settings of SRT |
| f12.4 | Five planners × three worlds | Nivo grouped bar (build-time data) | `tree_bench` CSV | hover bars | single- vs. multi-query economics; the undrivable Hitch path |
| — | Stale roadmap vs. fresh tree (hook) | animation (autoplay) | `sampling` + `robots` | replay | why single-query planners exist |

## 7. Exercises & Extensions

1. **(F)** Prove the Voronoi-bias lemma for the weighted $SE(2)$ metric
   $w_t\|X'-X''\|+w_r|\theta'-\theta''|_{S^1}$ and describe how the cells deform as $w_r\to0$;
   conclude what RRT does to the car's heading in that limit.
2. **(F)** C Alg. 13 swaps trees only after a successful first extension. Show that without the
   swap one tree can starve, and state the invariant (balanced node counts in expectation) the swap
   maintains.
3. **(C, predict-then-verify)** In w12.1 at $\eta=0.15$ rad the first solution takes about 900
   iterations with roughly 35% `Trapped`. Predict both numbers at $\eta=0.6$ rad *before* moving
   the slider, using the Voronoi overlay; verify, then find the $\eta$ minimizing iterations and
   explain why it is not the largest.
4. **(C)** In w12.2 set `Connect`/`Connect` and find a doorway width where it loses to
   `Extend`/`Extend` on median collision checks over 50 seeds; relate the crossover to the expected
   length of a greedy run before `Trapped`.
5. **(P)** Implement KPIECE's cell-importance selection over the joint-angle projection of Reach's
   torus in `kpiece.rs` (coverage, selections, neighbors in the importance formula — verify against
   the WAFR 2008 paper); add it to `tree_bench` and report where it beats RRT.
6. **(P, stretch)** Make `Sbl`'s lazy edge store resolution-aware (record the coarsest resolution at
   which an edge was checked, as Choset's lazy evaluation describes) and measure the savings over
   50 seeds when the goal moves slightly and the trees are reused.

## 8. Modernization Notes

- **Kept from Choset:** §7.2's two-family framing (EST pushes, RRT pulls) and all six algorithms
  (C Algs. 8–13) with their numbering; the metric-sensitivity and step-size discussion; the
  goal-bias caution, sharpened into derivation F.4; SBL's grid-cell $\pi_T$; §7.3's SRT in full —
  its parameter table is the best argument that Part III is one family, made interactive.
- **Labelled classical, not modern:** RRT-Connect [C ref. 249, Kuffner & LaValle], SBL [C ref.
  367, Sánchez & Latombe], Lazy PRM [C ref. 53], SRT [C ref. 14, 43], kinodynamic RRT [C ref. 270,
  271, LaValle & Kuffner].
- **Added beyond 2005:** (i) the Voronoi-bias lemma stated and drawn — Choset's text says RRT
  "pulls the tree to unexplored parts" and leaves the geometry to [C ref. 272]; (ii) the
  `ExtendStatus` contract and the blanket `Extend` over any Ch. 11 `Steer`, so Ch. 21's Dubins
  steer gets an RRT for free; (iii) KPIECE (Şucan & Kavraki, WAFR 2008) (verify) as the
  projection-based generalization of SBL's grid trick; (iv) the honest completeness citation — the
  rigorous RRT proof is Kleinbort et al. 2019 (verify), not the 2005 text; (v) the deliberate
  undrivable-Hitch benchmark row as the hook for Chapters 14 and 21.
- **Dropped or condensed:** RPP and Ariadne's clew (ancestry paragraphs); the $A^*_{cost}$-weighted
  $\pi_T$ of [C ref. 350] (a sentence — Ch. 13 does cost properly); parallel SRT (no implementation).
- **Honesty items the prose must keep:** RRT's first path is feasible, not good (Ch. 13 proves it
  never improves); step size and metric are tuning knobs with no theory behind their defaults; tree
  completeness proofs are harder than Choset's prose suggests and the text says where the real
  proof lives; the straight-line steer for Hitch is wrong and is shown anyway, labelled.
