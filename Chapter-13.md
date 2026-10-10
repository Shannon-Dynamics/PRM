# Chapter 13 — Optimal Sampling-Based Planning

> Part III — Sampling-Based Planning · Estimated length: 10 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Chapters 11 and 12 bought feasibility with randomness and proved the price: failure probability
decays exponentially. They said nothing about the *quality* of the path, and anyone who has watched
w12.1 has seen why — the first path is a jagged accident of sample order. This chapter asks the
question Choset's 2005 text could not: does more sampling make the path better? For RRT the
answer, proved by Karaman and Frazzoli in 2011, is *no, almost surely* — the tree is anchored to
its root through edges it never reconsiders. The repair is two local operations, choose-parent and
rewire, inside a neighborhood of radius $r_n=\gamma(\log n/n)^{1/d}$, and the "aha" is **why that
exact rate: the ball must shrink so the per-iteration cost stays $O(\log n)$, but no faster than
the scale at which a random geometric graph stays connected — Chapter 11's tiling balls, now
required to keep receiving samples forever.** From RRT\* and PRM\* the chapter builds the modern
batch family — Informed RRT\* samples only where improvement is possible, BIT\* and FMT\* search an
implicit random geometric graph with a heuristic — and ends with what "anytime" means for a robot
that must move before the planner is done. This is the canonical treatment of material the sister
book's Chapter 20 previews in one theorem; the implementations here are the reference ones.

Story line:
1. **Hook:** Reach executes its Ch. 12 RRT path — three needless reversals. Ten thousand more
   iterations: the path does not change. One flag flips the planner: the cost curve drops within
   the first thousand (autoplay).
2. **Play (C):** w13.1 Rewire Watch — RRT and RRT\* fed the same sample sequence; rewired edges
   flash; the cost curve descends toward the Ch. 6 lattice optimum.
3. **Formalize (F) and algorithms:** robust optimality; the random geometric graph and its radius;
   RRT's almost-sure suboptimality; PRM\*/RRT\* optimality with constants and the 2020 correction;
   the $(\epsilon,\alpha,\beta)$ lens; choose-parent and rewire; Informed RRT\*'s hyperspheroid;
   FMT\* lazy dynamic programming; BIT\* batches with a heuristic edge queue.
4. **Implementation (P):** `sampling::optimal` — `CostTree<M>`, `RrtStar`, `PrmStar`,
   `InformedRrtStar`, `Fmt`, `Bit`; the five-node rewire and $r_{1000}$ pinned by tests.
5. **Integration lab:** cost-vs-time for all five on Rusty/Apartment and Reach/Workbench against
   the lattice A\* floor; the anytime contract: stop at a deadline, execute, keep refining.
6. **Close:** optimality is asymptotic; Ch. 19's trajectory optimization is the complementary
   route — local, fast, no guarantee of the right homotopy class.

## 2. Prerequisites & Position

- **Builds on:** Ch. 11 (**imports everything**: `Sampler`, `KdTree::within` for the $r_n$-ball,
  `Steer`, `FreeSpace`, Theorem 7.4.1's tiling constant and the $(\epsilon,\alpha,\beta)$
  vocabulary — both reappear inside the optimality proofs), Ch. 12 (`Tree<M>`, `Extend`, RRT as
  the baseline; the Voronoi bias is unchanged by rewiring), Ch. 6 (A\* on the inflated lattice is
  the cost floor; its $g+h$ is BIT\*'s edge key), Ch. 5 (`Manifold::dist` as cost; the geodesic
  through the seam on $T^2$), Ch. 4 (the amber C-obstacle the ellipse is drawn over).
- **Feeds into:** Ch. 19 (trajectory optimization seeded by an RRT\* path; "asymptotic vs. local
  optimality"), Ch. 21 (Kinodynamic RRT\* replaces `Steer` with a two-point boundary-value solver),
  Ch. 22 (learned samplers inside Informed RRT\*'s rejection step), Ch. 23 (Reach's capstone
  planner is `RrtStar` with a deadline).
- **Baseline sources:** none in Choset — this chapter is NEW. It reuses C §7.4.1 (b. 243–246, the
  ball tiling), §7.4.2 (b. 246–250, expansiveness), §7.2.2 (b. 233–238, the RRT analyzed), §7.1.2
  (b. 211–214, shortcutting as the pre-2011 answer to path quality). Sister book: Ch. 20
  (`…/chapters/ch20-motion-planning`, states the Karaman–Frazzoli theorem in a paragraph and defers
  here), Ch. 23 (`…/chapters/ch23-mppi`, sampling-based *optimization* as the contrast).
  Modernization set (all **to verify**): Karaman & Frazzoli (IJRR 30(7), 2011); Solovey, Janson,
  Schmerling, Frazzoli & Pavone (ICRA 2020); Gammell, Srinivasa & Barfoot, Informed RRT\* (IROS
  2014), BIT\* (ICRA 2015; IJRR 2020); Janson, Schmerling, Clark & Pavone, FMT\* (IJRR 34(7), 2015).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; $n$, $k$, $r_n$ are book-wide):

| Symbol | Meaning |
|---|---|
| $c(\sigma)$, $c^*$, $c_n^{\mathrm{ALG}}$ | path cost (`dist`-length); optimal cost; best cost ALG holds after $n$ samples |
| $\mathrm{Cost}(v)$ | cost-to-come of tree node $v$ along its parent chain |
| $\zeta_d$ | volume of the unit ball in $\mathbb{R}^d$ ($\zeta_2=\pi$) |
| $\gamma$, $\gamma^*$, $k_n$ | radius constant; its critical value; $k_n=k^*\log n$ |
| $Q_{near}$ | $\{v:\mathrm{dist}(v,q_{new})\le r_n\}$ (or the $k_n$ nearest) |
| $\delta$-robust feasibility / optimality | a $\delta$-clear path exists / $c^*$ is the limit of such paths' costs |
| $X_{\hat f}$, $c_{best}$, $c_{min}$ | informed set; incumbent cost; $\mathrm{dist}(q_{start},q_{goal})$ |
| $\hat g$, $\hat h$, $\hat c$ | admissible estimates of cost-to-come, cost-to-go, edge cost |
| $G_n=(V_n,E_n)$ | random geometric graph on $n$ samples with radius $r_n$ |

**Definitions:**
- *Asymptotic optimality.* $\Pr[\lim_{n\to\infty}c_n^{\mathrm{ALG}}=c^*]=1$ for every robustly
  feasible problem with a robustly optimal solution.
- *Almost-sure suboptimality.* $\Pr[\lim_nc_n^{\mathrm{ALG}}=c^*]=0$ — the (monotone) limit exists
  and is wrong with probability one.
- *Choose-parent / rewire.* Attach $q_{new}$ to the $v\in Q_{near}$ minimizing
  $\mathrm{Cost}(v)+c(v,q_{new})$ subject to $\Delta$; then for each $v\in Q_{near}$ with
  $\mathrm{Cost}(q_{new})+c(q_{new},v)<\mathrm{Cost}(v)$ and $\Delta(q_{new},v)\ne$ NIL, set
  $\mathrm{par}(v):=q_{new}$ and propagate the decrease to $v$'s subtree.
- *Informed set.* $X_{\hat f}=\{x:\hat g(x)+\hat h(x)\le c_{best}\}$; for `dist` cost in
  $\mathbb{R}^d$ the prolate hyperspheroid with foci $q_{start},q_{goal}$, transverse diameter
  $c_{best}$, conjugate diameter $\sqrt{c_{best}^2-c_{min}^2}$.
- *Anytime planner.* Returns a feasible solution early and improves it monotonically until
  interrupted — "deadline in, best-so-far out".

**Derivations / theorems:**

1. **RRT is almost surely suboptimal (Karaman & Frazzoli 2011) (verify).** *Statement:* for
   $d\ge2$, `dist` cost, and the paper's regularity assumptions (robust feasibility, a robustly
   optimal path, finite $\mu(\mathcal{Q}_{free})$), $\Pr[\lim_nc_n^{\mathrm{RRT}}=c^*]=0$. *Sketch
   (5 steps):* (i) every tree path to the goal passes through one of the root's children; (ii) with
   positive probability the root's first children all leave at angles bounded away from the
   optimal path's initial direction; (iii) a later child must land in a root Voronoi cell that
   shrinks with $n$, so the probability of ever acquiring a child in the required cone is summable;
   (iv) Borel–Cantelli: almost surely only finitely many good children, and with positive
   probability none; (v) a path through a bad child costs $\ge c^*+\epsilon$ for a fixed $\epsilon$.
   *Collapsible:* the measure-theoretic statement; why shortcutting (C §7.1.2) does not rescue the
   limit — it stays in the homotopy class RRT committed to.
2. **PRM\*, RRG and RRT\* are asymptotically optimal (Karaman & Frazzoli 2011) (verify).**
   *Statement:* with $r_n=\gamma(\log n/n)^{1/d}$, $\gamma>\gamma^*=2(1+1/d)^{1/d}
   (\mu(\mathcal{Q}_{free})/\zeta_d)^{1/d}$ — or $k_n=k^*\log n$, $k^*>e(1+1/d)$ —
   $\Pr[\lim_nc_n=c^*]=1$, and expected per-iteration cost stays $O(\log n)$ because
   $\mathbb{E}|Q_{near}|=\Theta(\log n)$. *Sketch (7 steps):* (i) fix a robustly optimal path
   $\sigma^*$ with clearance $\delta$; (ii) tile it with balls of radius $\propto r_n$ as in
   Theorem 7.4.1 — but shrinking with $n$; (iii) the number of balls grows like $1/r_n$ and
   $\Pr[\text{ball empty}]=(1-\mu(B_{r_n}))^n\approx n^{-c\gamma^d}$; (iv) the union bound over
   balls is summable iff $c\gamma^d>1+1/d$ — this is where the constant comes from;
   (v) Borel–Cantelli: almost surely every ball is hit for all large $n$; (vi) consecutive balls
   are within $r_n$, so $G_n$ contains a path shadowing $\sigma^*$ with cost $\to c^*$; (vii) for
   RRT\*, choose-parent and rewire make the tree's cost-to-come match the RRG's shortest path along
   that shadow in the limit. *Collapsible:* the $\eta$ cap; the Ch. 11 lens —
   $\epsilon$-goodness makes the shadowing balls mutually visible, the same non-pathology
   Theorem 7.4.2 needed.
3. **The 2020 radius correction (Solovey, Janson, Schmerling, Frazzoli & Pavone) (verify).**
   *Statement:* the original RRT\* proof has a gap in step (vii) — rewiring need not reproduce the
   RRG shortest path because rewire only reconsiders nodes inside the *current* ball; a repaired
   argument establishes asymptotic optimality with a *smaller* constant,
   $\gamma>(2(1+1/d))^{1/d}(\mu(\mathcal{Q}_{free})/\zeta_d)^{1/d}$, and correspondingly for the
   $k$-nearest variant. *Sketch (4 steps):* (i) locate the gap; (ii) track a chain of tree nodes
   near $\sigma^*$ and bound the tree path's cost through them directly; (iii) each tiling ball must
   contain a sample *and* samples must arrive in an order that admits the chain — a per-ball
   probability; (iv) the union bound closes with the weaker constant. *Collapsible:* both constants side by
   side (the micro-example); the implementation names both, defaulting to the corrected one.
4. **Informed sampling is exactly where improvement is possible (Gammell et al. 2014) (verify).**
   *Statement:* with `dist` cost and admissible $\hat g,\hat h$, no configuration outside
   $X_{\hat f}$ lies on a path cheaper than $c_{best}$; sampling uniformly in $X_{\hat f}$ keeps
   RRT\*'s guarantees and multiplies the useful-sample probability by
   $\mu(\mathcal{Q}_{free})/\mu(X_{\hat f})$. *Sketch (4 steps):* (i) triangle inequality:
   $\mathrm{dist}(q_{start},x)+\mathrm{dist}(x,q_{goal})\le c(\text{any path through }x)$; (ii) so
   $x\notin X_{\hat f}$ ⇒ no improving path through $x$; (iii) in $\mathbb{R}^d$ sample a unit
   ball, scale by $\mathrm{diag}(c_{best}/2,\sqrt{c_{best}^2-c_{min}^2}/2,\dots)$, rotate by the
   $C$ aligning $e_1$ with $(q_{goal}-q_{start})/c_{min}$, translate to the midpoint; (iv) reject
   outside $\mathcal{Q}$; on $T^2$ use the geodesic through the seam and reject on the unwrapped
   cover. *Collapsible:* the measure ratio as "how much of the space is still worth sampling"
   (w13.2's readout); the ellipse is *necessary*, not sufficient — obstacles shrink the true set.
5. **FMT\* lazy dynamic programming is asymptotically optimal (Janson et al. 2015) (verify).**
   *Statement:* on one batch of $n$ samples with $r_n=\gamma(\log n/n)^{1/d}$,
   $\gamma>2(1/d)^{1/d}(\mu(\mathcal{Q}_{free})/\zeta_d)^{1/d}$, growing the tree outward in
   cost-to-come order and connecting each frontier node to its locally optimal parent *chosen
   ignoring obstacles, then checking only that edge* gives $\Pr[\lim c_n=c^*]=1$ with $O(n\log n)$
   expected collision checks. *Sketch (4 steps):* (i) the lazy choice is wrong only when the optimal
   in-ball parent is blocked; (ii) inside a $\delta$-clear corridor that event's probability
   vanishes with $r_n$; (iii) per-node error $O(r_n)$ over $O(1/r_n)$ nodes stays bounded and
   vanishes along a subsequence; (iv) one $k$-NN build, no rewiring. *Collapsible:* BIT\* (Gammell
   et al. 2015/2020) (verify): batches define an implicit RGG; edges expanded in order of
   $\hat g(v)+\hat c(v,x)+\hat h(x)$ exactly like Ch. 6's A\*, lazy checks on pop, pruning outside
   $X_{\hat f}$ after each solution, tree reuse across batches (an LPA\*-style repair).

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| RRT\*: choose-parent + rewire | `(T, q_new, Q_near) -> T'` | $O(|Q_{near}|)=O(\log n)$ steer calls; $O(n\log n)$ total; cost propagation $O(|\text{subtree}|)$ |
| PRM\* ($r_n$ or $k_n$) | `(n, γ) -> Roadmap` | $O(n\log n)$ expected edges |
| `r_n`, `gamma_star_kf2011`, `gamma_star_solovey2020`, `k_n` | `(usize, usize, f64) -> f64` | $O(1)$ |
| Informed RRT\* (verify) | `(q_start, q_goal, n, γ) -> Tree + cost curve` | RRT\* + $O(d^2)$ per sample |
| FMT\* (verify) | `(samples, r_n) -> Tree` | $O(n\log n)$ NN + lazy checks on chosen edges |
| BIT\* (verify) | `(batch, deadline) -> anytime Tree` | per batch: implicit RGG + heuristic edge queue |

**Numeric micro-example** (the contract with the unit test in §5). Euclidean plane, no obstacles,
a 5-node tree before one RRT\* iteration: root $A=(0,0)$, $\mathrm{Cost}=0$; $B=(3,0)$, parent $A$,
$\mathrm{Cost}=3$; $C=(3,3)$, parent $B$, $\mathrm{Cost}=6$; $D=(0,3)$, parent $A$,
$\mathrm{Cost}=3$. New sample $E=(1.5,1.5)$; neighborhood radius for this step $r=2.5$.
- **Near set.** $\mathrm{dist}(E,\cdot)=1.5\sqrt2=2.1213$ to each of $A,B,C,D$ — all four in $Q_{near}$.
- **Choose-parent.** Via $A$: $0+2.1213=2.1213$; via $B$ or $D$: $5.1213$; via $C$: $8.1213$.
  Parent $A$; $\mathrm{Cost}(E)=2.1213$.
- **Rewire.** Cost through $E$ to any neighbor: $2.1213+2.1213=4.2426=3\sqrt2$. $B$: $4.2426>3$,
  keep. $D$: keep. $C$: $4.2426<6$ → **rewire** $\mathrm{par}(C):=E$, $\mathrm{Cost}(C)$:
  $6\to4.2426$ (a 29.3% drop; $C$ has no children, so propagation is a no-op). Plain RRT would have
  added $E$ under $A$ and left $C$ at 6 forever.
- **Radius for $n=1000$, $d=2$**, $\mu(\mathcal{Q}_{free})=1$, $\zeta_2=\pi$:
  $(\log n/n)^{1/2}=(6.9078/1000)^{1/2}=0.08311$. $\gamma^*_{2011}=2\cdot1.5^{1/2}\pi^{-1/2}=1.3820$
  → $r_{1000}=0.1149$; $\gamma^*_{2020}=3^{1/2}\pi^{-1/2}=0.9772$ → $r_{1000}=0.0812$ (verify both
  against the papers). $k$-nearest: $k^*=e(1+1/2)=4.077$, $k_{1000}=4.077\cdot6.9078=28.2$ → 29.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"a tree allowed to change its mind, inside a ball that shrinks just slowly
enough."** Colors: tree **blue** with rewired edges flashing white-to-blue, incumbent path
**purple** (earlier incumbents fading), the informed ellipse outlined in **purple** too (a property
of the incumbent), start **green**, goal **red**, obstacles **slate**, C-obstacle **amber hatched**
on the torus, Reach **orange** during execution.

- **Widget w13.1: Rewire Watch** — type: wasm-sim (flagship). Split pane: RRT (left) and RRT\*
  (right) consuming the *same* seeded sample stream on the Apartment (Rusty as a disc) or Reach's
  torus (tab). Autoplays 3 000 iterations. **Manipulates:** headline — the radius multiplier
  $\gamma/\gamma^*$ (0.5 … 3; below 1 the hypothesis fails and the widget says so); disclosure —
  $r_n$ vs. $k_n$, 2011 vs. 2020 constant (verify), $\eta$ cap, pause-on-rewire, seed.
  **Observes:** rewired edges flash; cost vs. iteration for both planners with the Ch. 6 lattice
  A\* cost as a dashed floor; a histogram of $|Q_{near}|$ hovering near $\log n$; a subtree-
  propagation counter. **Misconception killed:** *"more samples make RRT's path better"* — the left
  curve is flat forever; the right descends.
- **Widget w13.2: Ellipse Shrinker** — type: wasm-sim. Informed RRT\* on the Apartment.
  **Manipulates:** headline — iterations after the first solution (scrubber); disclosure — informed
  sampling on/off (off = plain RRT\*), drag start/goal (foci move), seed. **Observes:** the ellipse
  tightening as $c_{best}$ drops; the readout $\mu(X_{\hat f})/\mu(\mathcal{Q})$; with informed
  sampling off, samples outside the ellipse drawn faintly as wasted. **Misconception killed:**
  *"after the first solution, every sample is equally useful."*
- **Widget w13.3: Batch Bench** — type: wasm-sim + Nivo line. RRT\*, Informed RRT\*, FMT\* (one
  batch of $n$), BIT\* (batches of 100) on the same seed and world. **Manipulates:** headline — a
  wall-clock deadline (50 ms … 5 s): the anytime contract made literal; disclosure — batch size,
  world tab, seed. **Observes:** cost vs. time per planner, time-to-first-solution markers, and the
  path each would *execute* if stopped at the deadline. **Misconception killed:** *"asymptotically
  optimal means better at every budget"* — FMT\* has nothing until its batch is done; BIT\*'s first
  batch beats RRT\* early; curves cross.

Dashboard layout: w13.1 full-width at the top of C, cost curve docked beneath, one iteration
scrubber for both panes; w13.2 half-width beside derivation F.4; w13.3 half-width at the end of P
as the lab's front end. All show their seed and degrade to SVG filmstrips plus the cost chart.

## 5. Practical (P) — Rust Implementation

**Crates:** `sampling` (Ch. 11 + Ch. 12's `tree` — all NN, steering, sampling and tree storage),
`nalgebra` 0.35 (the ellipse rotation via `SVD`), `rand` 0.9 `SmallRng`, `petgraph` 0.8 (PRM\*
roadmap), `pathfinding` 4.15 (the lattice A\* floor via Ch. 6 `search`). No new dependency.

**Module plan:** `crates/sampling/src/optimal/` (per TOC, a submodule of the Ch. 11 crate):
`mod.rs` (`CostTree<M>` wrapping Ch. 12's `Tree<M>` with `cost` and child lists); `radius.rs`
(`r_n`, both $\gamma^*$, `k_n`, `unit_ball_volume`); `rrt_star.rs` (`RrtStar`: `choose_parent`,
`rewire`, `propagate`; `Neighborhood`); `prm_star.rs` (`PrmStar` over Ch. 11's `Prm` with
`Connect::Radius(r_n)` / `KNearest(k_n)`); `informed.rs` (`InformedSampler: Sampler<M>` — drops
into any Ch. 11/12 planner — and `InformedRrtStar`); `fmt.rs` (`Fmt`: one batch, a cost-ordered
open heap, lazy parent choice); `bit.rs` (`Bit`: batches, an edge queue keyed
$\hat g+\hat c+\hat h$, pruning, tree reuse); `anytime.rs` (`Solution<P>` and `trait Anytime<P> { step, best, run_until(deadline) }` — one unit of
work per `step`, `Some` if improved; Ch. 23 consumes it);
`examples/rewire_five_nodes.rs`, `examples/cost_curves.rs`.

**Key types & signatures** (compiles in spirit; nothing from Chs. 11–12 is redefined):

```rust
use manifold::Manifold;
use sampling::{FreeSpace, Sampler, Steer, Chart, Uniform};                   // Ch. 11
use sampling::tree::{Tree, NodeId, Extend};                                 // Ch. 12
use rand::rngs::SmallRng;

/// r_n = γ (log n / n)^{1/d}. `gamma` is a parameter, not hidden, so the widget can violate the bound.
pub fn r_n(n: usize, d: usize, gamma: f64) -> f64 { gamma * ((n as f64).ln() / n as f64).powf(1.0 / d as f64) }
/// γ* = 2 (1 + 1/d)^{1/d} (μ_free / ζ_d)^{1/d}   — Karaman & Frazzoli 2011 (verify).
pub fn gamma_star_kf2011(d: usize, mu_free: f64) -> f64;
/// γ* = (2 (1 + 1/d))^{1/d} (μ_free / ζ_d)^{1/d} — Solovey et al. 2020 correction (verify).
pub fn gamma_star_solovey2020(d: usize, mu_free: f64) -> f64;
pub fn k_n(n: usize, d: usize) -> usize;                 // ⌈e (1 + 1/d) log n⌉

pub enum Neighborhood { Radius { gamma: f64, eta: f64 }, KNearest { k_star: f64 } }

/// Ch. 12's tree plus cost-to-come and child lists. Wrapping, not forking: `inner` is the same
/// arena the Ch. 12 widgets draw, so RRT and RRT* can be fed identical samples side by side.
pub struct CostTree<M: Chart> { pub inner: Tree<M>, pub cost: Vec<f64>, children: Vec<smallvec::SmallVec<[NodeId; 4]>> }
impl<M: Chart> CostTree<M> {
    pub fn near(&self, q: &M::Point, r: f64) -> Vec<(NodeId, f64)>;              // Ch. 11 KdTree::within
    pub fn add(&mut self, q: M::Point, parent: NodeId, edge_cost: f64) -> NodeId;
    pub fn reparent(&mut self, v: NodeId, new_parent: NodeId, edge_cost: f64);   // + propagate (DFS)
}

pub struct RrtStar<M: Chart, S: Extend<M>, Smp: Sampler<M>> {
    pub tree: CostTree<M>, pub steer: S, pub sampler: Smp, pub nbhd: Neighborhood,
    pub goal: M::Point, pub goal_radius: f64, pub rng: SmallRng,
    pub best: Option<(NodeId, f64)>, pub rewires: u64,                // counters the widget shows
}
impl<M: Chart, S: Extend<M>, Smp: Sampler<M>> RrtStar<M, S, Smp> {
    /// One iteration: sample → nearest → extend → near set → choose_parent → rewire → goal test.
    pub fn iterate(&mut self, free: &dyn FreeSpace<M>) -> Option<f64 /* new best cost */>;
    /// (parent, cost) minimizing Cost(v) + c(v, q_new) over `near` with Δ succeeding.
    fn choose_parent(&self, free: &dyn FreeSpace<M>, q_new: &M::Point, near: &[(NodeId, f64)]) -> Option<(NodeId, f64)>;
    /// For each v in `near`: if Cost(new) + c(new, v) < Cost(v) and Δ(new, v), reparent. Returns count.
    fn rewire(&mut self, free: &dyn FreeSpace<M>, new: NodeId, near: &[(NodeId, f64)]) -> usize;
}

/// Prolate hyperspheroid sampler (Gammell et al. 2014) (verify). It is a Ch. 11 `Sampler`, so
/// Informed RRT* is RRT* with a different `sampler` — no planner code changes.
pub struct InformedSampler<M: Chart> { start: M::Point, goal: M::Point, c_min: f64, pub c_best: f64,
    rotation: nalgebra::DMatrix<f64>, fallback: Uniform }
impl<M: Chart> InformedSampler<M> {
    pub fn update(&mut self, c_best: f64);
    pub fn measure_ratio(&self, mu_space: f64) -> f64;   // μ(X_f̂)/μ(Q): w13.2's readout
}
impl<M: Chart> Sampler<M> for InformedSampler<M> { /* unit ball → scale → rotate → translate → reject */ }

```

**Worked end-to-end example:** `cargo run --example rewire_five_nodes -p sampling` builds the §3
tree, inserts $E=(1.5,1.5)$ with a fixed radius $2.5$, and prints the near set (4 nodes at
`2.1213`), the choose-parent table (`A: 2.1213, B: 5.1213, D: 5.1213, C: 8.1213`), the rewire
decisions (`B keep, D keep, C rewire 6.0000 → 4.2426`), the resulting parent map, then
`r_n(1000, 2, γ*)` for both constants (`0.1149`, `0.0812`) and `k_n(1000, 2) = 29`.
`#[test] fn reproduces_five_node_rewire()` asserts parents and costs to $10^{-4}$;
`#[test] fn radius_constants()` pins both $r_{1000}$ values;
`#[test] fn rrt_star_cost_converges_under_fixed_seed()` runs `RrtStar` on the Apartment with
`SmallRng::seed_from_u64(13)` for 5 000 iterations and asserts a non-increasing cost sequence, a
final cost within 5% of the Ch. 6 lattice A\* floor, and that `Rrt` on the same samples has
constant cost after its first solution. `cost_curves` writes w13.3's cost-vs-time CSV.

**Runnable artifact:** the widgets run `web/lib/plan/{rrt_star,informed,fmt,bit}.ts`;
`lib/__checks__.ts` replays the five-node rewire and the radius constants. The Integration lab is
w13.3's deadline experiment on Rusty/Apartment and Reach/Workbench: stop at 100 ms, 500 ms, 2 s;
execute what you have (Reach's orange arm moves); keep planning and switch to the better path when
it arrives — the anytime loop the capstone (Ch. 23) reuses.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w13.1 | Rewire Watch | wasm-sim + Nivo line | `sampling::optimal` → `web/lib/plan/rrt_star.ts` + Ch. 12 `tree.ts` + `sim`; one seeded sample stream to both panes | $\gamma/\gamma^*$, $r_n$/$k_n$, 2011/2020 constant, $\eta$, pause-on-rewire, seed | RRT never improves; choose-parent + rewire; $|Q_{near}|\approx\log n$ |
| w13.2 | Ellipse Shrinker | wasm-sim | `web/lib/plan/informed.ts` | post-solution scrubber, informed on/off, drag foci, seed | the informed set is where improvement can happen |
| w13.3 | Batch Bench | wasm-sim + Nivo line | `web/lib/plan/{rrt_star,informed,fmt,bit}.ts` + worker | deadline, batch size, world, seed | anytime behavior; curves cross; batch vs. incremental |
| f13.4 | The shrinking tiling | static-svg (hover) | build-time plotters | hover $n$ → balls redrawn | why $r_n\propto(\log n/n)^{1/d}$ |
| — | Reach wobbles, then doesn't (hook) | animation (autoplay) | `sampling` + `robots` | replay | path quality is a separate contract |

## 7. Exercises & Extensions

1. **(F)** Carry out step (iv) of derivation F.2: with balls of radius $r_n/4$ tiling a path of
   length $L$, count the balls, write the union bound, and find the smallest exponent of $\gamma$
   for which $\sum_n\Pr[\text{some ball empty at }n]$ converges; identify where $(1+1/d)$ enters.
2. **(F)** Show that on $T^2$ the planar informed-set formula undercounts $\mu(X_{\hat f})$ when
   $c_{best}$ exceeds the torus circumference, and derive the correct sampler (union of ellipses
   over seam images, or rejection on the cover).
3. **(C, predict-then-verify)** In w13.1 on the Apartment at $\gamma/\gamma^*=1$, seed 13, RRT\*
   reaches 8% above the lattice floor by iteration 3 000. Predict, *before* touching the slider,
   what $\gamma/\gamma^*=0.5$ does — flat curve, slower descent, or the same? Verify, then explain
   via the expected size of $Q_{near}$.
4. **(C)** In w13.3 find the deadline at which BIT\* and RRT\* cross on the Workbench and the batch
   size that moves the crossing earliest; relate it to how many batches fit before the deadline.
5. **(P)** Implement `Neighborhood::KNearest` with $k_n=\lceil k^*\log n\rceil$ and a test that
   both neighborhoods reach the same 5 000-iteration cost within 3% under seed 13.
6. **(P, stretch)** Run Ch. 11's `shortcut_greedy` on RRT's path and plot, over 100 seeds, its cost
   against RRT\*'s at equal wall-clock time; find the two-homotopy-class map where it cannot win (F.1).

## 8. Modernization Notes

- **Kept from Choset (as scaffolding):** Theorem 7.4.1's ball tiling and Theorem 7.4.2's
  expansiveness constants reappear inside the optimality proofs, and the prose says so — the 2011
  theorems are Kavraki's 1996–1998 tiling with shrinking balls; §7.1.2's shortcutting is the
  pre-2011 answer to path quality, shown insufficient (exercise 6); Choset's "a large dense roadmap
  will probably yield shorter paths" (§7.1.2) is the intuition PRM\* turns into a theorem.
- **Added beyond 2005 (the whole chapter; every item to verify at authoring time):** Karaman &
  Frazzoli's RRT suboptimality and PRM\*/RRG/RRT\* optimality theorems with constants (IJRR 2011);
  the Solovey et al. radius correction (ICRA 2020) — both constants implemented and named; Informed
  RRT\* (IROS 2014); BIT\* (ICRA 2015; IJRR 2020); FMT\* (IJRR 2015); the `Anytime` interface.
- **Dropped or condensed:** RRT\#, LBT-RRT, SST and the rest of the post-2011 zoo (a pointer
  paragraph — the payload is the proof idea, not the taxonomy); Kinodynamic RRT\* deferred to Ch. 21.
- **Honesty items the prose must keep:** every theorem is asymptotic — at finite $n$ the cost is a
  random variable and the widget shows it; the hypotheses (robust optimality, `dist` cost, the
  $\gamma$ bound) are necessary and the widget lets the reader violate them; the 2011 RRT\* proof
  had a gap that took nine years to notice, and the chapter says so; the informed set is necessary,
  not sufficient; FMT\*'s time-to-first-solution is its batch; nothing here makes Hitch drivable.
