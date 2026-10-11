# Chapter 16 — Bayesian Localization and Mapping

> Part IV — Planning with Uncertainty · Estimated length: 7 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Chapter 15 gave the planner a Gaussian. This chapter takes it away. Rusty is switched on in the
Apartment without being told where; three doors look alike; the belief has three peaks and no mean
worth driving toward. Choset's Chapter 9 answers the estimation half — recursive Bayesian filtering
over *any* posterior, on a grid or by particles, a physical sensor model, occupancy-grid mapping,
Rao-Blackwellized SLAM — and the sister book treats every piece in depth. What neither writes down
is the half a *planner* needs: **what do Parts II–III do when $q_{start}$ is a distribution?** The
"aha": neither "plan from the mean" nor "solve a POMDP" but the loop every fielded robot runs —
**sample the belief, score each first action against a cost-to-go field computed once by backward
Dijkstra, take the cheapest in expectation, sense, update, repeat.** The Ch. 6 universal plan is
exactly what makes planning on a belief cheap. Choset's §9.1–9.2 are kept but compressed; the
chapter owns one artifact, `replan_on_belief`, which the capstone's navigation panel runs.

**Story line**
1. **Hook** — Rusty is kidnapped to a random room (autoplay). The Ch. 15 EKF, confident and wrong,
   drives the Part II planner into a wall; a uniform particle cloud finds the room in a dozen scans.
2. **Play (C)** — w16.1 Grid vs Particles: one logged run, two posteriors, cost meters ticking.
3. **Formalize (F)** — recursive Bayesian localization (9.1–9.15) derived once, its two independence
   assumptions named; three representations; Algs. 16–18; the sensor model (9.22–9.25); five cells.
4. **Mapping** — occupancy grids in log odds (9.26–9.39, Alg. 19); Bayesian SLAM by
   Rao-Blackwellization (9.43–9.48, Alg. 20) as a statement, the sister book carrying the machinery.
5. **The planner's view** — expected cost over samples, the backward Dijkstra value as cost-to-go,
   the collision penalty $\lambda$, the information bonus; the 3×3 example where mode and belief differ.
6. **Implementation (P)** — `estimate::bayes`: thin wrappers over the sister book's ported grid and
   particle localizers and occupancy grid; `replan_on_belief` on the Ch. 6 `search`.
7. **Integration lab** — w16.2 Plan-on-Belief: Rusty replans as the cloud condenses and the path
   bends toward the corridor whose walls disambiguate it. Close: the loop Ch. 23 inherits.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (Rusty, the Apartment, the ray-cast range sensor $\rho(x, s)$ — the sensor
  model is a likelihood over its readings), Ch. 6 (search, D\*, and the universal plan = backward
  Dijkstra, promoted from footnote to workhorse), Ch. 7 (the distance transform as a clearance
  cost), Ch. 15 (the Gaussian as one representation; the `estimate` crate this chapter extends).
- **Feeds into:** Ch. 19 (MPC consumes belief mean *and* covariance), Ch. 22 (learned heuristics on
  the same sampled-belief loop), Ch. 23 (the capstone's Rusty panel is `replan_on_belief` itself).
- **Baseline sources:** C §9.1 (b. 301–327): §9.1.1 (Fig. 9.1), §9.1.2 (eqs. 9.1–9.6), §9.1.3
  (9.7–9.15, two-rooms caveat), §9.1.4 (Kalman; grid with Alg. 16 and kernel 9.16; particles with
  Algs. 17–18, motion model 9.18–9.21, mean/mode 9.17, kidnapped-robot remedies), §9.1.5 (sensor
  model 9.22–9.25); C §9.2 (b. 328–345): §9.2.1 (9.26–9.42, Alg. 19), §9.2.2 (9.43, scan matching
  9.44, Rao-Blackwellization 9.45–9.48, Alg. 20). Sister cross-links
  (`https://prvr.shannon.id/chapters/<slug>`):
  `ch05-bayes-filter` (recursion, induction proof), `ch08-nonparametric-filters` (histogram and
  particle filters, KLD sizing), `ch10-sensor-models`, `ch12-localization-global` (grid, MCL,
  Augmented MCL, comparison table), `ch13-occupancy-grids`, `ch17-fastslam`, `ch21-mdps` (the value
  function our cost-to-go specializes), `ch22-pomdps` (planning on a belief done properly).
  Modernization set: Nav2's AMCL + costmap replanning loop (verify); QMDP — Littman, Cassandra &
  Kaelbling 1995 (verify); Prentice & Roy, belief roadmap (verify); Platt et al. (verify).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (Choset's bracket indices continue from Ch. 15; planner symbols are the
book's):

| Symbol | Meaning |
|---|---|
| $P(x(k) \mid u(0{:}k{-}1), y(1{:}k))$ | the posterior over configurations after $k$ steps; $= bel(x_t)$ |
| $P(x(k) \mid u(k{-}1), x(k{-}1))$, $P(y(k) \mid x(k))$ | motion model, sensor model; the map $m$ is background knowledge |
| $\eta(k)$; $\mathcal{M} = \{(x_j, \omega_j)\}_{j=1}^N$ | normalizer computed on the fly (9.6); particle set with importance weights |
| $(\alpha, \beta, d)$ | Choset's odometry increment: initial rotation, translation, final rotation (Fig. 9.5) |
| $d(x)$, $h_{i,j}$ | ray-cast expected distance; sensor-model histogram bin (expected $d_i$, reading $d_j$) |
| $m_l$, $\operatorname{Odds}(\cdot)$ | occupancy of cell $l$; odds $P/(1-P)$ |
| $J(c)$ | cost-to-go of lattice cell $c$ to $q_{goal}$ — the Ch. 6 universal plan (backward Dijkstra) |
| $\mathcal{J}_{bel}(a) = \sum_j \omega_j\,[\,1 + J(f(x_j, a))\,]$; $\lambda$, $\beta_{info}$ | expected one-step-plus-cost-to-go of $a$ under the sampled belief; collision penalty, information bonus |

The planner's $q$ and the filter's $x(k)$ coincide for Rusty; a grid belief lives on the Ch. 6
lattice, so $J$ and the belief share cells. Colors: beliefs in the sister's `--pr-*` roles; planning
roles for what the planner draws (`--rm-goal`, `--rm-obstacle`, `--rm-graph` for the cost-to-go
field, `--rm-path`, `--rm-robot`). Since `--pr-posterior` and `--rm-path` are both purple-family,
w16.2 draws the posterior as filled cells/dots and the path as a thick haloed stroke.

**Definitions** (display boxes):
- *Position tracking / global localization / kidnapped robot* (§9.1): initial configuration known
  approximately / unknown / believed wrongly and to be unlearned. *Posterior representation*: a
  finite approximation of $P : X \to \mathbb{R}$ — Gaussian, grid over $SE(2)$, or weighted samples.
- *Sensor model*: $P(y \mid x) = P(y \mid d(x))$ (9.22), a mixture of hit, short, random, and
  max-range components (9.23). *Occupancy grid*: independent binary cells, $P(m) = \prod_l P(m_l)$.
- *Belief-replanning policy*: $a^* = \arg\min_a \mathcal{J}_{bel}(a)$, recomputed after every
  sense–update; an expectation-of-cost rule, not a POMDP policy.

**Derivations / theorems**

1. **Recursive Bayesian localization, derived once** (§9.1.3, eqs. 9.7–9.15). *Statement:*
   $P(x(k) \mid u(0{:}k{-}1), y(1{:}k)) = \eta(k)\, P(y(k) \mid x(k)) \sum_{x(k-1)} P(x(k) \mid u(k{-}1), x(k{-}1))\,
   P(x(k{-}1) \mid u(0{:}k{-}2), y(1{:}k{-}1))$. *Sketch:* (i) Bayes rule with
   $u(0{:}k{-}1), y(1{:}k{-}1)$ as background (9.7); (ii) **assumption A**: given $x(k)$, $y(k)$ is
   independent of earlier data (9.8); (iii) the denominator is $\eta(k)$ (9.9); (iv) total
   probability over $x(k{-}1)$ (9.10); (v) **assumption B**: given $x(k{-}1)$ and $u(k{-}1)$,
   $x(k)$ is independent of older data (9.11); (vi) drop $u(k{-}1)$ from the previous belief's
   conditioning (9.13) — valid for small motions; Choset's two-rooms counterexample shows when not;
   (vii) substitute. *Collapsible:* the continuous form (9.2); C problem 2 (the KF from it); the
   sister's induction proof at `…/ch05-bayes-filter`.
2. **The normalizer is free** (9.5–9.6). *Statement:*
   $\eta(k) = [\sum_{x(k)} P(y(k) \mid x(k))\, P(x(k) \mid u(0{:}k{-}1), y(1{:}k{-}1))]^{-1}$.
   *Sketch:* (i) $\eta^{-1}$ is the marginal likelihood of $y(k)$; (ii) total probability over
   $x(k)$; (iii) every term already appears in the update — one running sum (Alg. 16 lines 6–13).
   *Collapsible:* this is the $\tilde p$ the kidnapped-robot monitors of §9.1.4 threshold.
3. **Log-odds occupancy recursion** (§9.2.1, eqs. 9.26–9.36). *Statement:*
   $\log\operatorname{Odds}(m \mid x(1{:}k), y(1{:}k)) = \log\operatorname{Odds}(m \mid x(k), y(k))
   - \log\operatorname{Odds}(m) + \log\operatorname{Odds}(m \mid x(1{:}k{-}1), y(1{:}k{-}1))$.
   *Sketch:* (i) Bayes rule on $m$ with the pose history as background (9.26); (ii) $y(k)$ depends
   only on $m, x(k)$ (9.27); (iii) Bayes again to swap $P(y \mid m, x)$ for the inverse model
   $P(m \mid x, y)$ (9.28–9.29); (iv) the same for $\neg m$, divide — the hard denominator cancels
   (9.30–9.31); (v) logs; prior $0.5$ kills the prior term (9.36). *Collapsible:* (9.40), the cone
   model (9.41–9.42), Alg. 19; the independence trap and MAP repair at sister `ch13-occupancy-grids`.
4. **Rao-Blackwellization of the SLAM posterior** (§9.2.2, eqs. 9.45–9.47). *Statement:*
   $P(x(1{:}k), m \mid \cdot) = P(m \mid x(1{:}k), y(1{:}k))\, P(x(1{:}k) \mid y(1{:}k), u(0{:}k{-}1))$:
   a map term in closed form given the path (derivation 3) times a path term a particle filter
   samples. *Sketch:* (i) chain rule (9.45); (ii) $m$ is independent of $u$ given the path (9.46);
   (iii) each particle carries a path and its own map (9.48); (iv) weight = likelihood of $y(k)$
   under that map (Alg. 20 line 11); (v) resample. *Collapsible:* tree-shared maps [C ref. 321,
   346]; scan matching (9.44) and its global inconsistency (Figs. 9.19–9.20); sister `ch17-fastslam`.
5. **Replanning on a belief as expected cost-to-go** (this book's addition). *Statement:* let $J$
   be the exact cost-to-go on the free lattice from backward Dijkstra (Ch. 6), with
   $J(\text{obstacle}) = J(\text{off-grid}) = \lambda$. For a belief given by weighted samples,
   $a^* = \arg\min_a \sum_j \omega_j [\,1 + J(f(x_j, a))\,]$ minimizes the expected cost of "take
   $a$, then follow the optimal plan from wherever you actually are"; the certainty-equivalent choice
   $\arg\min_a [1 + J(f(x_{MAP}, a))]$ coincides with it iff all samples agree. *Sketch:* (i) with
   known state the optimal first action is $\arg\min_a [1 + J(f(x,a))]$ by Bellman's principle;
   (ii) with unknown state and no further observations, moving the expectation outside the $\min$ is
   the QMDP approximation (verify); (iii) $J$ is computed once per goal, so a decision costs
   $O(K \cdot |A|)$ after one $O(|V| \log |V|)$ Dijkstra; (iv) replanning after each sense–update
   restores the observations the one-shot rule ignored. *Collapsible:* the information bonus
   $-\beta_{info}\,\Delta H_a$ (expected one-step entropy reduction from the sensor model at the
   sampled poses) — what makes the path "hug information"; this is a heuristic for the POMDP of
   sister `ch22-pomdps`; Ch. 6's D\* replaces the full Dijkstra when only the map changes.

**Named algorithms**

| Algorithm | Signature | Complexity |
|---|---|---|
| C Alg. 16 `grid_localization` | `(bel, u(0:k−1), y(1:k)) -> bel` | $O(N^2)$ naive; $O(N W)$ with shift + separable kernel (9.16) |
| C Alg. 17 `particle_localization` | `(M, u(0:k−1), y(1:k)) -> M` | $O(N)$ plus one sensor-model evaluation per particle |
| C Alg. 18 `resample` (low-variance) | `(M) -> M'` | $O(N)$ |
| C Alg. 19 `occupancy_grid_mapping` | `(y(1:k), x(1:k), P₀(m)) -> P(m)` | $O(\text{cells in cone})$ per scan |
| C Alg. 20 `rbpf_slam` | `(M, u(0:k−1), y(1:k)) -> M` | $O(N \cdot \text{scan})$ with incremental maps |
| `cost_to_go` (Ch. 6 backward Dijkstra, imported) | `(grid, goal, λ) -> J` | $O(|V| \log |V|)$ once per goal |
| `replan_on_belief` (owned) | `(J, samples, actions, λ, β_info) -> Action` | $O(K \cdot |A|)$ per decision |

**Numeric micro-example** (the contract with the unit tests in §5), in two parts.

*Part A — five-cell grid localization, sense then move.* A cyclic corridor of cells $\{0..4\}$ with
doors at 1 and 3 (Choset problem 6's numbers): $P(\text{door} \mid \text{door cell}) = 0.8$,
$P(\text{door} \mid \text{wall cell}) = 0.4$; "right 1" succeeds with $0.8$, stays with $0.2$
(problem 7). Uniform prior $0.2$. **Sense** door: unnormalized $[0.08, 0.16, 0.08, 0.16, 0.08]$,
$\eta^{-1} = 0.56$, posterior $[\tfrac17, \tfrac27, \tfrac17, \tfrac27, \tfrac17] \approx
[0.1429, 0.2857, 0.1429, 0.2857, 0.1429]$; entropy $2.3219 \to 2.2359$ bits. **Move** right:
$bel'(c) = 0.8\,bel(c{-}1) + 0.2\,bel(c)$ gives $\tfrac17[1.0, 1.2, 1.8, 1.2, 1.8] \approx
[0.1429, 0.1714, 0.2571, 0.1714, 0.2571]$, entropy $\to 2.2811$ bits — sensing sharpened, moving
smeared. The test asserts every entry to $10^{-4}$.

*Part B — the replan decision on a 3×3 grid.* Rows top-to-bottom $0..2$, 4-connected, unit step
cost, obstacle at $(1,0)$, goal $(0,1)$. Backward Dijkstra: $J(0,1)=0$; $J(0,0)=J(0,2)=J(1,1)=1$;
$J(1,2)=J(2,1)=2$; $J(2,0)=J(2,2)=3$. Belief: $x_1 = (2,1)$, $\omega_1 = 0.6$ (the mode) and
$x_2 = (2,0)$, $\omega_2 = 0.4$. Penalty $\lambda = 10$.
- Mode-only plan from $(2,1)$: **Up** ($1 + J(1,1) = 2$).
- Belief plan: $\mathcal{J}_{bel}(\text{Up}) = 0.6 \cdot 2 + 0.4 \cdot 10 = 5.2$ (Up from $(2,0)$
  hits the obstacle); $\mathcal{J}_{bel}(\text{Right}) = 0.6 \cdot 4 + 0.4 \cdot 3 = 3.6$;
  $\mathcal{J}_{bel}(\text{Left}) = 0.6 \cdot 4 + 0.4 \cdot 10 = 6.4$; Down is off-grid for both:
  $10$. **Right** wins: the planner first moves *sideways* to a column safe under both hypotheses.
- Break-even: $0.6 \cdot 2 + 0.4\lambda = 3.6 \Rightarrow \lambda^* = 6$; below it the planner gambles
  on the mode. The test asserts the four costs at $\lambda = 10$, the tie at $6$, the flip at $4$ (Up $= 2.8$).

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, one line: **"don't drive from where you probably are — drive so that wherever you
are, the rest of the trip stays cheap."**

- **Widget w16.1: Grid vs Particles** — type: wasm-sim. One recorded Apartment run (seeded odometry
  and scans from the Ch. 2 sim) replayed into two localizers: a grid over $SE(2)$ (Alg. 16 with the
  shift-and-convolve prediction 9.16) and a particle filter (Algs. 17–18) started uniform over free
  space. **Manipulates:** headline — one "budget" slider moving grid resolution and particle count
  together along equal-cost settings; behind a disclosure — decouple them, kidnap button,
  sensor-mixture weights, beam subsampling (Choset's 60-of-181 remark), seed re-roll. **Observes:**
  both posteriors in `--pr-posterior`, truth gray dashed; a live cost meter (ms, bytes) under each;
  mean (9.17) and mode as two markers, so the mean visibly falls inside a wall during the bimodal
  phase; the $\tilde p$ strip the kidnap recovery thresholds. **Misconception killed:** *"the
  particle filter is just a cheaper grid"* — at equal cost they fail differently: the grid blurs,
  the cloud starves, and only one recovers from a kidnap without a restart.
- **Widget w16.2: Plan-on-Belief** — type: wasm-sim (flagship; the owned artifact). Rusty, a
  particle belief, the goal in `--rm-goal`, the cost-to-go field shaded `--rm-graph`, the chosen path
  in `--rm-path`. Each tick: sample $K$ particles, score actions, execute the best, sense, update.
  **Manipulates:** headline — the collision penalty $\lambda$ (the §3 break-even marked on the
  slider); behind a disclosure — $K$, $\beta_{info}$, "plan from the mode" and "freeze belief"
  toggles, seed re-roll. **Observes:** with the mode toggle on, the path drives confidently through a
  doorway that exists only under the mode, and bumps; with it off, the first moves are sideways, the
  cloud collapses as corridor walls come into view, and the path straightens; with $\beta_{info} > 0$
  the path detours toward distinctive geometry and the entropy sparkline drops sooner.
  **Misconception killed:** *"planning on a belief means planning from its mean"* — the mean can be
  in a wall and the mode can be wrong; expected cost over samples is the honest minimum short of the POMDP.
- **Figure f16.3: The Five-Cell Corridor** — type: static-svg (hover). Choset's Fig. 9.1 shrunk to
  the §3 micro-example — prior, sense (green likelihood strip), posterior, move — with the exact
  sevenths and entropies printed; hover reveals the sister's Hallway Belief Machine link.

Dashboard layout: w16.1 full-width at the head of the C section, cost meters docked beneath; f16.3
half-width beside derivation 1, sticky while it scrolls; w16.2 full-width closing the chapter with an
inset reproducing the 3×3 decision table live as $\lambda$ moves. All widgets autoplay a seeded
default (seed visible) and degrade to pre-rendered SVG filmstrips.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35, `rand` 0.9 (`Pcg64`; the resampler draws one uniform per step as in
Alg. 18), `parry2d` 0.30 via the Ch. 2 `sim` ray-cast for $d(x)$, `petgraph` 0.8 and the Ch. 6
`search` crate for the lattice and backward Dijkstra, and the sister book's ported crates —
`HistogramFilter`, `ParticleFilter`, `low_variance_resample` (its Ch. 8), `GridLocalizer`, `Mcl`,
`AugmentedMcl` (its Ch. 12), `OccGrid` (its Ch. 13), `GridRbpf` (its Ch. 17) — vendored unchanged in
the `filters`/`localize` ports Ch. 15 opened. Nothing here re-implements a filter.

**Module plan** — `crates/estimate/src/bayes/` (extends the Ch. 15 crate): `mod.rs` (re-exports;
`trait Belief`); `corridor.rs` (the five-cell example as the sister's `HallwayFilter<5>` with Choset
problem 6–7 parameters — no new filter code); `sensor.rs` (`ProximityModel`, eq. 9.23, the 256×256
histogram cache of §9.1.5, `fit_ml`); `odometry.rs` (the $(\alpha, \beta, d)$ increment as a
`MotionModel` adapter over the Ch. 2 encoder stream); `replan.rs` (**owned artifact**: `cost_to_go`,
`replan_on_belief`, `BeliefPlanner`, `InfoBonus`); `examples/{five_cells,replan_3x3,kidnap}.rs`;
`demos/ch16-widgets/`.

**Key types & signatures**

```rust
// crates/estimate/src/bayes/mod.rs
pub use filters::nonparametric::{HistogramFilter, ParticleFilter, ParticleSet, low_variance_resample};
pub use filters::localize::{GridLocalizer, Mcl, AugmentedMcl};
pub use filters::occgrid::OccGrid;
use search::{Lattice, CellIdx, Action};  use rand_pcg::Pcg64;   // Ch. 6 grid and its moves

/// What a planner needs from any posterior: weighted samples on the planning lattice plus the two
/// statistics Choset discusses. Implemented for the grid belief, the particle set, and the Ch. 15
/// Gaussian, so `replan_on_belief` is representation-agnostic.
pub trait Belief {
    fn samples(&self, k: usize, lattice: &Lattice, rng: &mut Pcg64) -> Vec<(CellIdx, f64)>;
    fn mode(&self, lattice: &Lattice) -> CellIdx;
    fn entropy_bits(&self) -> f64;
}

// crates/estimate/src/bayes/sensor.rs — Choset (9.23): hit N(y; d, σ) + short λe^{-λy} +
// random γ + max-range δ, binned at Δ; d(x) comes from the Ch. 2 ray-cast, never re-implemented.
pub struct ProximityModel { pub sigma: f64, pub lambda: f64, pub alpha: f64, pub beta: f64,
                            pub gamma: f64, pub delta: f64, pub delta_bin: f64, pub max_range: f64 }
impl ProximityModel {
    pub fn likelihood(&self, y: f64, d_expected: f64) -> f64;     // cached h_{i,j}
    /// Product over a subsampled scan (9.25); `every` = Choset's 60-of-181 trick.
    pub fn scan_likelihood(&self, scan: &sim::Scan, expected: &sim::Scan, every: usize) -> f64;
}

// crates/estimate/src/bayes/replan.rs
/// K draws per decision; collision / off-grid penalty λ; information bonus weight (0 disables).
pub struct ReplanConfig { pub k_samples: usize, pub lambda: f64, pub beta_info: f64 }
/// Ch. 6 universal plan: exact cost-to-go to `goal`, obstacles and off-grid set to λ. Computed
/// once per goal; D* (Ch. 6) repairs it when the map changes.
pub fn cost_to_go(lattice: &Lattice, goal: CellIdx, lambda: f64) -> search::ValueField;

/// a* = argmin_a Σ_j ω_j [ 1 + J(f(x_j, a)) ] − β_info · ΔH_a. Returns the per-action table too:
/// the widget's inset and the §3 micro-example test both need it.
pub fn replan_on_belief(
    j: &search::ValueField, samples: &[(CellIdx, f64)], lattice: &Lattice,
    cfg: &ReplanConfig, info: Option<&dyn InfoBonus>,
) -> (Action, Vec<(Action, f64)>);

/// Expected one-step entropy reduction at a pose, from the sensor model — a heuristic, named so.
pub trait InfoBonus { fn expected_entropy_drop(&self, at: CellIdx, a: Action) -> f64; }

/// The loop every fielded robot runs: sense → update → replan → act; one loop for grid, cloud, or
/// the Ch. 15 Gaussian. `step` acts against the Ch. 2 sim, reads odometry and scan, updates.
pub struct BeliefPlanner<B: Belief> {
    pub belief: B, pub j: search::ValueField, pub cfg: ReplanConfig, pub rng: Pcg64,
}
impl<B: Belief> BeliefPlanner<B> {
    pub fn decide(&mut self, lattice: &Lattice) -> Action;
    pub fn step(&mut self, lattice: &Lattice, robot: &mut sim::Rusty, world: &sim::World) -> Action;
}
```

**Worked end-to-end example:** `cargo run --example five_cells -p estimate` builds the
`HallwayFilter<5>` instance (doors $\{1, 3\}$, $0.8/0.4$, kernel $[0.2, 0.8, 0.0]$), runs sense then
move, and prints the §3 Part A table; `#[test] fn reproduces_five_cell_sense_move()` pins it.
`cargo run --example replan_3x3` builds the lattice with the obstacle at $(1,0)$, prints $J$ as a
3×3 table, then the four expected costs at $\lambda \in \{10, 6, 4\}$ with the chosen action;
`#[test] fn replan_disagrees_with_mode_at_lambda_10()` asserts $\{5.2, 3.6, 6.4, 10\}$, the tie at
$6$, the flip at $4$; `#[test] fn point_mass_belief_equals_a_star()` checks that one sample of weight
1 reproduces the Ch. 6 A\* path step by step on fifty seeded Apartment queries. `examples/kidnap.rs`
prints the step at which each w16.1 localizer concentrates $> 0.9$ of its mass within 0.5 m of truth.

**Runnable artifact:** `estimate` with `bayes` compiles to WASM as w16.1 and w16.2; the $\lambda$
slider sets `ReplanConfig::lambda` and the inset table *is* the `Vec<(Action, f64)>` returned by
`replan_on_belief`. The Integration lab kidnaps Rusty in the Ch. 2 Apartment, localizes with
`AugmentedMcl` fed by `ProximityModel` and the odometry adapter, and navigates with `BeliefPlanner`
over the Ch. 6 lattice inflated by the Ch. 7 brushfire clearance — the stack the capstone runs.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w16.1 | Grid vs Particles | wasm-sim | estimate::bayes + filters/localize (ports) + sim, seeded Pcg64 | budget slider, decouple, kidnap, mixture weights, beam subsampling, seed re-roll | same recursion, two representations; mean vs. mode; failure modes at equal cost |
| w16.2 | Plan-on-Belief | wasm-sim | estimate::bayes::replan + search (Ch. 6) + potential::brushfire (Ch. 7) + localize + sim | λ slider, K, β_info, plan-from-mode, freeze-belief, seed re-roll | expected cost over samples beats planning from mean/mode; replanning restores lost information |
| f16.3 | The Five-Cell Corridor | static-svg (hover) | build-time SVG | hover each stage | sensing sharpens, moving smears, with exact fractions |
| — | Kidnap hook | animation (autoplay) | estimate + sim | replay only | a confident Gaussian drives into a wall; a cloud recovers |

## 7. Exercises & Extensions

1. **(F)** Choset problem 6: ten places on a circle, landmarks at $\{0, 3, 6\}$, likelihoods
   $0.8/0.4$, deterministic moves of 3 then 4 counterclockwise, observations landmark / landmark /
   none. Compute the posterior by hand; redo it with problem 7's $0.8/0.2$ succeed/stay motion.
2. **(F)** Choset problem 8: show $y(k)$ is independent of $y(1{:}k{-}1)$ given $x(k)$ and $m$, and
   exhibit the dependence when $m$ is unknown; relate it to why Bayesian SLAM carries a map per path.
3. **(C, predict-then-verify)** In w16.2 with the default belief (two clusters near $0.6/0.4$) and
   $\lambda = 10$, predict the first action and the $\lambda$ at which it flips using the §3
   break-even on the inset table. Verify both; then lower $K$ until the decision turns noisy and
   report the $K$ at which the flip point moves by more than one slider step.
4. **(C)** In w16.1, kidnap Rusty during the bimodal phase and predict which wall the mean marker
   lands in before it is drawn; explain via 9.17 why the circular heading mean can be meaningless.
5. **(P)** Implement `Belief` for the Ch. 15 `Gaussian<3>` by sigma points (seven for $n = 3$)
   instead of seeded draws; property-test that for a tight Gaussian `replan_on_belief` and A\* from
   the mean agree on 100 seeded queries while for a wide one they disagree at least once.
6. **(P, stretch)** Replace the full backward Dijkstra in `cost_to_go` with the Ch. 6 D\* Lite so an
   occupancy change repairs only affected cells; measure `BeliefPlanner::step` time, fixed seed,
   before and after on a run with twenty door events.

## 8. Modernization Notes

- **Kept, and why:** Choset's §9.1.3 derivation with its *two* named assumptions and the two-rooms
  caveat — the planner must know when the recursion is a lie; Algorithms 16–20 with their numbers;
  the $(\alpha, \beta, d)$ odometry increment, which maps onto the Ch. 2 encoder stream; the
  four-component sensor model with ML fit and ray-cast cache; the log-odds recursion;
  Rao-Blackwellized SLAM as 2005's state of the art — Murphy and coworkers [C ref. 137, 329],
  Montemerlo et al. [C ref. 321], Hähnel, Burgard, Fox, and Thrun [C ref. 181]; the kidnapped-robot
  remedies of Fox et al. [C ref. 156], Burgard et al. [C ref. 82], Lenser and Veloso [C ref. 275],
  Gutmann and Fox [C ref. 176].
- **Added beyond 2005:** (i) the planner's view — `replan_on_belief` and the expected-cost-to-go
  argument, which neither Choset nor the sister book states as a planning primitive; named honestly
  as the QMDP approximation (verify), below the belief roadmap of Prentice & Roy (verify) and Platt
  et al. (verify), with the POMDP treatment at sister `ch22-pomdps`; (ii) the Ch. 6 universal plan
  promoted to the object that makes belief planning cheap; (iii) the information bonus as an
  explicit, switchable heuristic; (iv) Nav2's AMCL-plus-costmap loop (verify) as the fielded
  instance; (v) depth linked, not re-derived: induction proof (sister Ch. 5), resampling variance
  and KLD sizing (Ch. 8), beam intrinsics and likelihood fields (Ch. 10), Augmented MCL and the
  comparison table (Ch. 12), the independence trap and MAP mapping (Ch. 13), FastSLAM (Ch. 17).
- **Dropped or condensed, and why:** the AAAI-94 and Freiburg experiment narratives (Figs. 9.3–9.4,
  9.7–9.8, 9.17–9.23) — replaced by live Apartment runs; the full cone model (9.41–9.42) — a
  collapsible, the sister book owning the implementation; topological discretizations and the
  Jensfeld–Christensen mixture [C ref. 209] — one sentence each; scan matching (9.44) — a pointer.
- **Honesty items the prose must keep:** assumption B fails for large moves (two rooms); per-cell
  independence of occupancy grids and per-beam independence (9.25) are both false and both useful;
  the belief planner is a one-step expectation with replanning, not an optimal POMDP policy, and its
  answer depends on $\lambda$ (the break-even is printed, not hidden); the information bonus is a
  heuristic; the five-cell numbers use Choset's problem parameters, not a figure from his text.
