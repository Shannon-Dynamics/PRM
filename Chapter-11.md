# Chapter 11 — Probabilistic Roadmaps

> Part III — Sampling-Based Planning · Estimated length: 11 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Part II built roadmaps by *constructing* $\mathcal{Q}_{free}$ — visibility graphs from obstacle
vertices, Voronoi diagrams from equidistance, cells from critical points — and every one of those
methods needs an explicit description of the free space. Choset's Figure 7.1, a ten-degree-of-
freedom arm threading clutter, is the problem none of them can touch. This chapter makes the turn
that defined the field after 1996: stop describing $\mathcal{Q}_{free}$ and *ask it questions*. A
collision checker answers "is $q$ free?" in microseconds; a roadmap of coarse random samples
(nodes) and fine sampling along local paths (edges) captures the connectivity of a space nobody
wrote down. The "aha": **a planner can be complete in a weaker, honest sense — probabilistically —
and the weakness is quantifiable: PRM's failure probability decays *exponentially* in the sample
count, with constants computable from path length and clearance.** The sister book's Chapter 20
previews PRM in a dozen lines; this chapter is the canonical treatment it cross-links to, and it
owns the sampling machinery (samplers, nearest-neighbor structure, `trait Steer`) that Chapters
12, 13, and 21 import.

Story line:
1. **Hook:** Reach (3R mode) must pass its elbow through a slot on the Workbench. Ch. 8's
   visibility graph has no vertices; Ch. 10's torus decomposition is thousands of cells. Six hundred
   random samples and a straight-line local planner solve it on autoplay.
2. **Play (C):** w11.1 Roadmap Rain — samples fall, edges connect, the component counter drops,
   and the frame where start and goal join one component freezes on screen.
3. **Formalize (F):** learning/query phases; local planner $\Delta$ and `dist`; C Algs. 6–7; what
   "probabilistically complete" means as a limit statement.
4. **Make it practical, then analyze it:** metrics on $T^2$ and $SE(2)$, kd-trees with periodic
   axes, the straight-line local planner and its two collision schedules, shortcutting; the sampler
   zoo for narrow passages and the connection strategies (Lazy PRM, classical); then Theorems
   7.4.1–7.4.3 — ball tiling, $(\epsilon,\alpha,\beta)$-expansiveness, abstract path tiling.
5. **Implementation (P):** the `sampling` crate — `Sampler`, `NearestNeighbors` + `KdTree`,
   `Steer` + `StraightLine`, `Prm<M>` generic over the Ch. 5 `Manifold`; a six-sample roadmap
   reproduced by a unit test.
6. **Integration lab:** PRM for Rusty (disc, Apartment) and Reach (torus, Workbench) with the same
   code; success rate vs. corridor width in w11.2; close on what a roadmap cannot do — answer *one*
   query fast — which is Chapter 12's job.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (`Collision` over parry2d — the only question asked of the world; the three
  worlds), Ch. 4 ($\mathcal{QO}_i$ as a set never computed here; Reach's torus), Ch. 5 (`trait
  Manifold` with `dist`/`interpolate`/`sample` on $S^1$, $T^2$, $SE(2)$; App. C.2 metrics — the
  kd-tree is built over these), Ch. 6 (Dijkstra/A\* in the query phase), Ch. 8 (roadmap properties
  accessibility/connectivity/departability, now held with high probability; the GVD as the target
  of medial-axis sampling), App. D (completeness notions).
- **Feeds into:** Ch. 12 (adds `extend` to `Steer`; EST/RRT/SBL/SRT reuse `Sampler`, `KdTree`),
  Ch. 13 (RRT\*/PRM\* import everything and add costs and radii; their constants use
  $\mu(\mathcal{Q}_{free})$ exactly as Theorem 7.4.1's $\sigma$ does), Ch. 14 (composite manifolds
  plug into the same `Prm`), Ch. 21 (Dubins/Reeds–Shepp implement `Steer`), Ch. 22 (learned
  samplers implement `Sampler`), Ch. 23 (Reach's roadmap in the capstone).
- **Baseline sources:** C §7 intro (b. 197–202), §7.1.1 (b. 203–208: Basic PRM, Algs. 6–7),
  §7.1.2 (b. 208–215: uniform sampling, kd-trees, distance functions and embeddings, local
  planner, incremental vs. subdivision checking, postprocessing), §7.1.3 (b. 216–225: OBPRM,
  Gaussian, bridge test, medial-axis retraction, visibility PRM, quasirandom sequences,
  discrepancy/dispersion, connection sampling), §7.1.4 (b. 225–227: sparse roadmaps, lazy
  evaluation), §7.4.1–7.4.3 (b. 243–253: Theorems 7.4.1–7.4.3, Defs. 7.4.1–7.4.5, Alg. 15).
  Sister book: Ch. 20 (`…/chapters/ch20-motion-planning`, the preview this chapter deepens), Ch. 8
  (`…/chapters/ch08-nonparametric-filters`, sampling as a representation of a set you cannot write
  down), Ch. 3 (`…/chapters/ch03-geometry-of-motion`, $SE(2)$ interpolation). Modernization set:
  `kiddo` nearest-neighbor crate (candidate; verify metric hooks and WASM); ANN is already Choset's
  [C ref. 30, Arya & Mount].

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; $n$, $k$, $r_n$, $(\epsilon,\alpha,\beta)$ are book-wide):

| Symbol | Meaning |
|---|---|
| $\Delta(q, q')$ | local planner: a collision-free local path from $q$ to $q'$, or NIL |
| $\mathrm{dist}(q,q')$, $\mathrm{emb}(q)$ | distance function on $\mathcal{Q}$; embedding of $q$ by $p$ robot points into $\mathbb{R}^{p\cdot\dim\mathcal{W}}$ |
| $N_q$ | the $k$ closest neighbors of $q$ in $V$ under `dist` |
| $\mathrm{clr}(\gamma)$, $L$ | clearance and length of a known path $\gamma$ |
| $\mu(A)$, $B_\rho(x)$, $\sigma$ | volume; open ball; $\sigma = \mu(B_1)/(2^d\mu(\mathcal{Q}_{free}))$ |
| $\mathrm{reach}(S)$, $\mathrm{lookout}_\beta(S)$ | visibility set of $S$; its $\beta$-lookout |
| $xRy$, $R^\ell$ | abstract local-planner relation; its $\ell$-fold iterate |
| $D(P,\mathcal{R})$, $\delta(P,\rho)$ | discrepancy and dispersion of a point set $P$ |

**Definitions** (definition boxes):
- *Probabilistic roadmap; learning and query phases.* Undirected $G=(V,E)$, $V\subset\mathcal{Q}_{free}$
  sampled, $(q,q')\in E$ iff $q'\in N_q$ and $\Delta(q,q')\ne$ NIL; built by C Alg. 6, queried by
  C Alg. 7 (connect $q_{start},q_{goal}$, then search); the phases may interleave.
- *Probabilistic completeness* (C §7.4). For every solvable $(a,b)$,
  $\lim_{n\to\infty}\Pr[(a,b)\ \text{FAILURE}]=0$; resolution completeness is the deterministic
  analogue, relative to the sampling resolution.
- *Clearance.* $\mathrm{clr}(\gamma)=\sup\{\rho:B_\rho(\gamma(t))\subset\mathcal{Q}_{free}\ \forall t\}$.
- *$\epsilon$-good, $\beta$-lookout, $(\epsilon,\alpha,\beta)$-expansive* (C Defs. 7.4.2–7.4.4):
  every point sees $\ge\epsilon\mu(\mathcal{Q}_{free})$; $\mathrm{lookout}_\beta(S)$ is the part
  of $S$ seeing $\ge\beta$ of the rest of its component; expansive if every connected $S$ has
  $\mu(\mathrm{lookout}_\beta(S))\ge\alpha\mu(S)$.
- *Linking sequence* (C Def. 7.4.5): $x_1=x$, $x_i\in\mathrm{lookout}_\beta(X_{i-1})$, $X_i=X_{i-1}\cup\mathrm{reach}(x_i)$.
- *Discrepancy, dispersion* (C Defs. 7.1.1–7.1.2); Sukharev: $\delta\ge1/(2\lfloor N^{1/d}\rfloor)$
  under $L_\infty$, so $N\ge(1/2\delta^*)^d$ — dispersion costs exponentially in $d$.

**Derivations / theorems:**

1. **Theorem 7.4.1 — PRM in $\mathbb{R}^d$.** *Statement:* if a path $\gamma$ of length $L$ and
   clearance $\rho>0$ joins $a,b\in\mathcal{Q}_{free}\subset[0,1]^d$, then after $n$ uniform samples
   $\Pr[\text{FAILURE}]\le\lceil2L/\rho\rceil\,e^{-\sigma\rho^dn}$. *Sketch (6 steps):* (i) place
   $m=\lceil2L/\rho\rceil$ points along $\gamma$ with consecutive distance $<\rho/2$;
   (ii) $y_i\in B_{\rho/2}(x_i)$, $y_{i+1}\in B_{\rho/2}(x_{i+1})$ both lie in the convex free ball
   $B_\rho(x_i)$, so the segment is free and the straight-line $\Delta$ succeeds; (iii) one sample
   per ball yields a roadmap path; (iv) union bound over $m$ empty-ball events; (v) each has
   probability $(1-\sigma\rho^d)^n$; (vi) $(1-\beta)^n\le e^{-\beta n}$. *Collapsible:* why the
   $k$-nearest rule can break step (ii) (the gap Ch. 13's PRM\* radius closes); the
   varying-clearance extension [C ref. 223, Kavraki, Kolountzakis & Latombe].
2. **Theorem 7.4.2 — expansive spaces.** *Statement:* if $\mathcal{Q}_{free}$ is
   $(\epsilon,\alpha,\beta)$-expansive and $2n+2$ uniform samples are drawn with
   $n=\lceil8\ln(8/\epsilon\alpha\delta)/\epsilon\alpha+3/\beta\rceil$, then with probability
   $\ge1-\delta$ every induced subgraph $G_i$ is connected. *Sketch (6 steps):* (i) Lemma 7.4.6:
   with $s=1/\alpha\epsilon$, $x\in V$ has a length-$t$ linking sequence in $V$ with probability
   $\ge1-se^{-(n-t-1)/s}$; (ii) Lemma 7.4.7: $t\ge\beta^{-1}\ln4$ gives
   $\mu(X_t)\ge\tfrac34\mu(\mathcal{Q}_{free,i})$; (iii) two such sets overlap in $\ge\tfrac12$;
   (iv) a third batch hits the overlap with probability $\ge1-e^{-n\epsilon/2}$; (v) union bound
   over $\binom n2$ pairs; (vi) solve for $n$. *Collapsible:* Figure 7.19's estimate
   $\epsilon\approx\alpha\approx\beta\approx w/W$ for two squares joined by a corridor of width $w$
   — what makes w11.2's success meter predictable.
3. **Theorem 7.4.3 — abstract path tiling.** *Statement:* with any measurable relation $R$ as
   local planner and any sampling measure $\mu$, PRM is probabilistically complete iff C Alg. 15
   (guess $x_{\ell+1}\sim\mu$, require $x_\ell Rx_{\ell+1}$, stop when $x_\ell Ry$) succeeds with
   nonzero probability on every query; then $\Pr[\text{FAILURE}]\le\ell e^{-pn}$. *Sketch (5
   steps):* (i) forward: Alg. 15's path is a PRM path under the same $\mu$; (ii) converse: a minimal
   successful roadmap of $n$ nodes is permutation-invariant, so Alg. 15 succeeds with probability
   $\ge P/n!$; (iii) decompose the success set $S\subset R^\ell$ into rectangles and pick one of
   positive product measure; (iv) $p=\min_j\mu(A^i_j)$; (v) union bound: $\ell(1-p)^n$.
   *Collapsible:* symmetry and reflexivity were never used — asymmetric local planners and time as
   a coordinate (Ch. 14's preview, Ch. 21); Ladd & Kavraki's framework [C ref. 252].
4. **Where Gaussian and bridge samplers put their mass.** *Statement:* the Gaussian sampler's
   density is the probability that a $\sigma$-ball around $q$ straddles $\partial\mathcal{QO}$,
   concentrating within $O(\sigma)$ of boundaries; the bridge test concentrates in corridors
   narrower than the bridge. *Sketch (4 steps):* (i) acceptance = integral of the collision
   indicator over the partner distribution; (ii) for a half-space obstacle it is
   $\Phi(-\mathrm{dist}/\sigma)$; (iii) both piers in collision: a corridor of width $w$ admits
   bridges of length $\ell>w$ in a large fraction of directions, open space in almost none;
   (iv) OBPRM's bisection lands within one `step` of the boundary. *Collapsible:* medial-axis
   retraction [C ref. 427, Wilmarth, Amato & Stiller] and the Ch. 8 GVD; hybrids
   $\pi=(1-w)\pi_A+w\pi_B$ and samplers as filters (C §7.1.3); the kd-tree's periodic-axis
   pruning bound $\min(|c-s|,T-|c-s|)$ and its $O(dn\log n)$ build / $O(n^{1-1/d}+m)$ query
   [C ref. 124, de Berg, van Kreveld & Overmars].

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| C Alg. 6 Roadmap Construction (BASIC PRM) | `(n, k, sampler, dist, Δ) -> G` | $n$ samples + $n$ kNN queries + $\le nk$ calls to $\Delta$ |
| C Alg. 7 Solve Query | `(q_start, q_goal, k, G) -> Option<Path>` | $\le2k$ calls to $\Delta$ + one Dijkstra/A\* (Ch. 6) |
| Uniform / Gaussian / Bridge / OBPRM / Medial-axis samplers | `(space, free, rng) -> Option<q>` | 1 / 2 / 3 / bisection / retraction checks per attempt |
| `kd_build`, `kd_knn`, `kd_range` | `(points)`; `(q, k)`; `(q, r)` | $O(dn\log n)$; $O(n^{1-1/d}+k)$; $O(n^{1-1/d}+m)$ |
| Straight-line local planner (incremental / subdivision) | `(q, q', step) -> Option<LocalPath>` | $\lceil\mathrm{dist}/\text{step}\rceil$ checks; subdivision exits early |
| Connection sampling (C §7.1.3) | `(G, w) -> G'` | never creates components |
| Lazy PRM (classical, [C ref. 53, Bohlin & Kavraki]) | `(G_unchecked, q_start, q_goal) -> Option<Path>` | checks nodes then edges on candidate paths, coarse-to-fine |
| C Alg. 15 Random Incremental | `(x, y, μ, R) -> Option<Path>` | analysis device |

**Numeric micro-example** (the contract with the unit test in §5). Point robot in
$\mathcal{W}=[0,10]^2$, one wall $\mathcal{WO}=[4.5,5.5]\times[0,7]$ (so $\mathcal{QO}=\mathcal{WO}$;
gap above $y=7$). Six free samples $s_1=(1,2)$, $s_2=(3,5)$, $s_3=(2,8)$, $s_4=(7,2)$, $s_5=(8,6)$,
$s_6=(6,9)$; Euclidean `dist`; straight-line $\Delta$ with subdivision at step $0.25$. Distances:
$d_{12}=\sqrt{13}=3.606$, $d_{23}=\sqrt{10}=3.162$, $d_{36}=d_{45}=\sqrt{17}=4.123$, $d_{56}=3.606$,
$d_{14}=6.000$, $d_{24}=d_{26}=5.000$, $d_{25}=5.099$.
- **$k=1$:** candidates $\{s_1s_2,s_2s_3,s_4s_5,s_5s_6\}$, all free → **2 components**
  $\{s_1,s_2,s_3\}$, $\{s_4,s_5,s_6\}$: the roadmap has not seen the gap.
- **$k=2$:** adds $s_1s_4$ (crosses the wall at $y=2$: NIL), $s_2s_4$ (crosses at
  $y\in[3.125,3.875]$: NIL), $s_3s_6$ (passes the wall's $x$-band at $y\in[8.625,8.875]>7$: free).
  Edges $s_1s_2,s_2s_3,s_3s_6,s_5s_6,s_4s_5$ → **1 component**; roadmap path $s_1\to s_4$ has length
  $3.606+3.162+4.123+3.606+4.123=18.620$.
- **Theorem 7.4.1 with concrete constants:** $d=2$, $\mu(\mathcal{Q}_{free})=0.9$ of the unit
  square, $L=1.2$, $\rho=0.1$: $\sigma=\pi/(4\cdot0.9)=0.8727$, $\sigma\rho^2=0.008727$, $m=24$;
  $\Pr[\text{FAILURE}]\le24e^{-0.008727n}$: $0.306$ at $n=500$, $0.0039$ at $n=1000$; $n\ge892$
  guarantees $\le1\%$. Theorem 7.4.2 at $\epsilon=\alpha=\beta=0.1$, $\delta=0.05$ asks for
  $n=7775$, i.e. $2n+2=15{,}552$ samples — right shape, hopeless constants; the prose says so.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"coarse rain for nodes, fine rain for edges"** — PRM samples at two
resolutions and every design choice is about where each rain falls. Colors: start **green**, goal
**red**, workspace obstacle **slate**, C-obstacle **amber hatched** (Reach's torus view), roadmap
**blue** with one tint per component until they merge, solution **purple**, Rusty/Reach **orange**.

- **Widget w11.1: Roadmap Rain** — type: wasm-sim (flagship). Tabs: Rusty as a disc in the
  Apartment ($\mathbb{R}^2$, inflated walls) and Reach on the Workbench ($T^2$, unwrapped square
  with the Ch. 4 amber C-obstacle). Autoplays a seeded build to $n=600$, $k=6$. **Manipulates:**
  headline — a sample-count scrubber $n$ (the build is deterministic in the seed, so scrubbing is
  replay); disclosure — $k$, sampler strategy, edge `step` and check schedule (the numbered check
  order of Choset's Fig. 7.6 shown on the hovered edge), drag start/goal, seed re-roll with the seed
  shown. **Observes:** component counter and a components-vs-$n$ sparkline; new edges drawn in the
  tint of the component they join; the frame at which $q_{start}$ and $q_{goal}$ first share a
  component frozen as a thumbnail; the purple Dijkstra path; separate "local planner calls" and
  "collision checks" counters so the two rains are measured apart. **Misconception killed:** *"PRM
  is a random graph, so whether it connects is luck"* — the component curve collapses the same way
  on every re-roll; luck lives in constants, not shape.
- **Widget w11.2: Narrow Passage** — type: wasm-sim. Two rooms joined by a corridor, Rusty as a
  disc. **Manipulates:** headline — corridor width $w$ in disc radii; disclosure — sampler chips
  (uniform, Gaussian, bridge, OBPRM, medial-axis; multi-select), $n$, seed. **Observes:** for each
  strategy, 100 seeded roadmaps run in a worker and the connect fraction is plotted live vs. $w$
  (Nivo line, one series per strategy); the active strategy's sample cloud shows *where* it puts
  mass; a readout of $\epsilon\approx w/W$ beside the measured rate. **Misconception killed:**
  *"uniform sampling sees everything at practical sample counts"* — and its twin, *"biased sampling
  is always better"* (Gaussian loses to uniform in the open room).

Dashboard layout: w11.1 full-width at the top of C with the component sparkline docked beneath;
w11.2 half-width beside the expansiveness definitions in F (its $w/W$ readout points at Theorem
7.4.2). Both autoplay seeded, show their seed, and degrade to build-time SVG.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`DVector<f64>` chart coordinates), `parry2d` 0.30 only through the
Ch. 2 `collide` crate, `petgraph` 0.8 (`UnGraph` roadmap, `UnionFind`), `rand` 0.9 (`SmallRng`;
`rand_distr::Normal` for the Gaussian sampler), `kiddo` as a *candidate* cross-check for flat
$\mathbb{R}^d$ in tests only (verify WASM and metric hooks; the book's tree is hand-rolled because
$T^2$ and $SE(2)$ need periodic axes), `pathfinding` 4.15 cross-checks the Ch. 6 Dijkstra.

**Module plan:** `crates/sampling/` — **introduced here, imported by Chs. 12–14, 21, 22.**
`src/lib.rs` (re-exports; `FreeSpace<M>` adapter over Ch. 2's `Collision`); `src/sampler.rs`
(`Sampler`; `Uniform`, `Gaussian`, `Bridge`, `Obprm`, `MedialAxis`, `Hybrid`, `Scripted` — the last
replays a list for tests and Ch. 12's traced example); `src/nn.rs` (`NearestNeighbors`; `Linear`
oracle, `KdTree` with periodic axes); `src/steer.rs` (`Steer`, `StraightLine`, `EdgeCheck`,
`LocalPath`); `src/prm.rs` (`Prm`, `Roadmap`, `Connect::{KNearest, Sparse, Lazy}`, connection
sampling, `shortcut_greedy`); `src/analysis.rs` (`prm_failure_bound`, `expansive_sample_count`:
pure functions behind the F numbers and w11.2's readout); `examples/{six_sample_prm,narrow_passage_sweep}.rs`.

**Key types & signatures** (compiles in spirit; `Manifold` is Ch. 5's and is *not* redefined):

```rust
use manifold::Manifold;                 // Ch. 5: type Point; dim, dist, interpolate, sample
use rand::rngs::SmallRng;

/// What this chapter asks of the world — and nothing else. Blanket-implemented for any
/// (robot, world) pair with a Ch. 2 `Collision` impl, so planners never see parry2d.
pub trait FreeSpace<M: Manifold> {
    fn is_free(&self, q: &M::Point) -> bool;
    fn checks(&self) -> u64;                    // collision-check counter: the book's cost unit
}

/// Node sampler. `None` = attempt rejected (Gaussian/bridge/OBPRM reject most attempts by design).
pub trait Sampler<M: Manifold> {
    fn sample(&mut self, space: &M, free: &dyn FreeSpace<M>, rng: &mut SmallRng) -> Option<M::Point>;
}
pub struct Uniform;  pub struct Gaussian { pub sigma: f64 }  pub struct Bridge { pub length: f64 }   // + Obprm, MedialAxis, Hybrid, Scripted

/// Coordinates for kd-tree splitting; `period(axis)` is Some(2π) on S¹ factors.
/// The metric stays Manifold::dist — the chart only decides how to split and prune.
pub trait Chart: Manifold {
    const D: usize;
    fn coords(&self, q: &Self::Point) -> nalgebra::DVector<f64>;   // not SVector<f64, {Self::D}>: unstable
    fn period(&self, axis: usize) -> Option<f64>;
}
pub trait NearestNeighbors<M: Manifold> {
    fn insert(&mut self, idx: usize, q: M::Point);
    fn k_nearest(&self, q: &M::Point, k: usize) -> Vec<(usize, f64)>;   // sorted by dist
    fn within(&self, q: &M::Point, r: f64) -> Vec<(usize, f64)>;        // Ch. 13 needs this
}

pub enum EdgeCheck { Incremental { step: f64 }, Subdivision { step: f64 } }   // C §7.1.2
pub struct LocalPath<M: Manifold> { pub waypoints: Vec<M::Point>, pub length: f64 }

/// Δ(q, q'): the local planner. `StraightLine` is deterministic and symmetric; Ch. 21's
/// Dubins/Reeds–Shepp impls are asymmetric and the roadmap becomes directed (C §7.1.1).
pub trait Steer<M: Manifold> {
    fn steer(&self, space: &M, free: &dyn FreeSpace<M>, from: &M::Point, to: &M::Point)
        -> Option<LocalPath<M>>;
    fn symmetric(&self) -> bool { true }
}
pub struct StraightLine { pub check: EdgeCheck }

pub enum Connect { KNearest { k: usize }, Sparse { k: usize, redundant: usize }, Lazy { k: usize } }
pub struct Prm<M: Manifold, S: Sampler<M>, N: NearestNeighbors<M>> {
    pub space: M, pub sampler: S, pub nn: N, pub steer: Box<dyn Steer<M>>, pub connect: Connect,
    pub roadmap: Roadmap<M>,          // petgraph UnGraph<M::Point, f64> + UnionFind
    pub rng: SmallRng,                // seeded; the widget shows the seed
}
impl<M: Manifold, S: Sampler<M>, N: NearestNeighbors<M>> Prm<M, S, N> {
    /// C Alg. 6. Resumable: a larger `n` extends the existing roadmap (C §7.1.1 "Adding").
    pub fn build(&mut self, free: &dyn FreeSpace<M>, n: usize);
    /// C Alg. 7: connect both endpoints to one component, then Ch. 6 Dijkstra.
    pub fn query(&mut self, free: &dyn FreeSpace<M>, q_start: &M::Point, q_goal: &M::Point, k: usize)
        -> Option<Vec<M::Point>>;
}
```

**Worked end-to-end example:** `cargo run --example six_sample_prm -p sampling` builds the §3
roadmap with `Scripted` replaying $s_1\dots s_6$ over `cspace::R2`, `StraightLine` with
`Subdivision { step: 0.25 }`, and prints for $k=1$ and $k=2$ the candidate edges, the two $\Delta$
rejections ($s_1s_4$, $s_2s_4$), the edge set, the component count (2, then 1) and the
$s_1\to s_4$ path length `18.620`; then `prm_failure_bound(2, 0.9, 1.2, 0.1, n)` for
$n\in\{500,1000\}$ (`0.306`, `0.0039`) and the smallest $n$ with bound $\le0.01$ (`892`).
`#[test] fn reproduces_six_sample_roadmap()` asserts edge sets exactly and lengths to $10^{-3}$;
`#[test] fn kd_tree_matches_linear_on_torus()` is a property test (2 000 $T^2$ points, 200 queries,
seam-crossing neighbors included); `kiddo_agrees_in_r2` cross-checks the flat case (verify crate).
`narrow_passage_sweep` writes w11.2's curves (5 strategies × 8 widths × 100 seeds).

**Runnable artifact:** the widgets run the TypeScript port `web/lib/plan/{prm,sampler,kdtree}.ts`,
pinned to the six-sample roadmap in `lib/__checks__.ts`; a WASM build of `sampling` runs in CI to
assert port and canonical produce identical roadmaps from one seed. The Integration lab drops the
same `Prm` into the Apartment (Rusty, $\mathbb{R}^2$) and the Workbench (Reach, $T^2$) with zero
planner code changed — only `Manifold` and `FreeSpace` — and times one query against the build.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w11.1 | Roadmap Rain | wasm-sim | `sampling` (canonical) → `web/lib/plan/prm.ts` + `sim` worlds + SimCanvas, seeded SmallRng | $n$ scrubber, $k$, sampler chips, step and check schedule, drag start/goal, seed | two-resolution sampling; components merging; connectivity as process, not luck |
| w11.2 | Narrow Passage | wasm-sim + Nivo line | `sampling` → `web/lib/plan/{prm,sampler}.ts`, worker pool | corridor width, strategy multi-select, $n$, seed | narrow passages; where biased samplers put mass; $\epsilon\approx w/W$ |
| f11.3 | kd-tree over the torus | static-svg (hover) | build-time plotters + CSS hover | hover a cell → its subtree | periodic split planes; the seam wrap test |
| — | Reach through the slot (hook) | animation (autoplay) | `sampling` + `robots` + SimCanvas | replay | explicit geometry fails, 600 samples succeed |

## 7. Exercises & Extensions

1. **(F)** Redo Theorem 7.4.1 for the $k$-nearest rule: exhibit samples where $k=1$ misses the
   tiling although every ball is hit, and state the smallest change (radius rule, or $k$ growing
   with $n$) that restores the proof — the gap Chapter 13's PRM\* radius closes.
2. **(F)** For two squares of side $W$ joined by a corridor of width $w$ and length $W$, verify
   Figure 7.19's $\epsilon\approx\alpha\approx\beta\approx w/W$ from $\mu(\mathrm{reach}(x))$ at mid-corridor.
3. **(C, predict-then-verify)** In w11.2 at $n=500$, uniform sampling connects about 85% of seeds
   at $w=1.0$ radii. Predict the rate at $w=0.5$ *before* moving the slider from
   $\epsilon\approx w/W$ and Theorem 7.4.2's shape; then enable the bridge test and explain from
   derivation F.4 why its curve is nearly flat in $w$.
4. **(C)** In w11.1's torus tab, find a seed and $n$ where two components interleave on the
   unwrapped square, separated only by the amber band; say which connection strategy joins them
   soonest and why.
5. **(P)** Implement `Connect::Lazy` (Lazy PRM, [C ref. 53]): build with no edge checks, search,
   check nodes then edges along the candidate path coarse-to-fine, delete failures, re-search.
   Report collision checks vs. `KNearest` on the Workbench over 50 seeds.
6. **(P, stretch)** Add a `Halton` sampler (bases 2 and 3 on the torus) and a dispersion estimator;
   show seed-independence at fixed $n$, then build the adversarial layout Choset promises exists.

## 8. Modernization Notes

- **Kept from Choset, in full:** §7.1's two-phase PRM with Algs. 6–7 as numbered; the §7.1.2
  implementation guidelines (metrics, embeddings, kd-trees, incremental vs. subdivision checks,
  shortcutting); the §7.1.3 sampler zoo including quasirandom sequences, discrepancy, dispersion
  and Sukharev's bound; all three §7.4 analyses with proofs sketched honestly — this is the book's
  canonical treatment of probabilistic completeness, so nothing in §7.4 is skipped.
- **Labelled classical, not modern:** Lazy PRM [C ref. 53, 54, Bohlin & Kavraki], the bridge test
  [C ref. 193, Hsu, Jiang, Reif & Sun], OBPRM [C ref. 18, Amato et al.], the Gaussian sampler
  [C ref. 59, Boor, Overmars & van der Stappen], MAPRM [C ref. 427], visibility PRM [C ref. 337,
  Nissoux, Siméon & Laumond], ANN [C ref. 30] — all in Choset's bibliography.
- **Added beyond 2005:** (i) the planner is *generic over the Ch. 5 `Manifold`* — one `Prm` runs on
  $\mathbb{R}^2$, $T^2$, $SE(2)$ unchanged, and the kd-tree handles periodic axes exactly rather
  than via embedding; (ii) collision checks and local-planner calls as first-class counters, so
  every comparison in the book is in Choset's own cost unit; (iii) `kiddo` as a named candidate,
  hooks to verify, periodic case hand-rolled; (iv) the pointer that the $k$-nearest rule leaves a
  gap in Theorem 7.4.1 which Ch. 13's $r_n$/$k_n$ rules close; (v) learned samplers (Ch. 22).
- **Dropped or condensed:** manipulability-based sampling (an exercise on Ch. 4's Jacobian),
  workspace-GVD sampling on graphics hardware (pointer), the 3D rigid-body example of §7.1.2
  (replaced by Reach's torus; the quaternion-sampling caution points at Ch. 5's `SO(3)`).
- **Honesty items the prose must keep:** exponential in shape, hopeless in constants (15 552
  samples in the micro-example); a finite-`step` edge check can miss a collision, and only the
  exact distance-based schedule of [C ref. 376] cannot; PRM can report "no path" falsely; no
  sampler wins everywhere and pathological instances exist for every one (C §7.1.3, close).
