# Chapter 19 — Trajectory Optimization and MPC

> Part V — Dynamics, Trajectories, and Constraints · Estimated length: 11 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Chapter 18 fixed the path and optimized the clock. This chapter lets the path move too — and discovers
that, once a trajectory is a vector of numbers, "planning" becomes "descend a cost." Choset gave this
two pages of optimal control and three of nonlinear programming, with the warning that the result
depends on the initial guess. Twenty years later that warning is the whole discipline: CHOMP, STOMP,
TrajOpt, iLQR, and model predictive control are five answers to *which* descent, in *which* metric,
with obstacles as *cost* or *constraint*. The "aha": **a trajectory optimizer is a local planner with a
global vocabulary** — it polishes the homotopy class it is handed, brilliantly, and never changes its
mind. So the pipeline keeps Part III's sampling planners for the global decision and uses this
chapter's machinery to make that decision smooth, fast, dynamically feasible, and closed-loop.

Story line:
1. **Hook** — a jagged Ch. 12 RRT path for Reach, replayed; then the same path after thirty CHOMP
   iterations: shorter, smooth, hugging the obstacle at a chosen clearance, and executable by the Ch. 18
   time scaler in half the time.
2. **Play (C)** — w19.1 CHOMP Smoother: drag the step size, watch the relaxation; pick the other initial
   path and watch the optimizer converge to a *different* answer.
3. **Formalize (F)** — the historical root: Pontryagin's minimum principle, the Hamiltonian, the
   double-integrator examples (min-effort is linear, min-time is bang-bang — Ch. 18's result from another
   door), shooting; transcription to a nonlinear program.
4. **Algorithm** — the modern family: CHOMP's covariant functional gradient over a distance field,
   STOMP's gradient-free stochastic variant, TrajOpt's sequential convex optimization with signed-
   distance constraints, iLQR/DDP's backward value recursion, MPC as the loop around any of them.
5. **Implementation (P)** — the `trajopt` crate: the shared GN/LM optimizer, `chomp`, `stomp`,
   `sco`, `ilqr`, `mpc`; `argmin` as the comparison library; `potential` (Ch. 7) for $D(x)$,
   `dynamics` (Ch. 17) for iLQR's model.
6. **Integration lab** — the Reach pipeline Ch. 23 ships: RRT → shortcut → CHOMP → time scale → replay;
   Rusty under receding-horizon iLQR with a scripted moving obstacle. **Close:** where optimization beats
   search (smoothness, dynamics, speed) and where it sticks (homotopy, narrow passages) — on to Chs. 20–21.

## 2. Prerequisites & Position

- **Builds on:** Ch. 7 (`DistanceField` with $\nabla D$ — CHOMP's obstacle term; §4.7's $J^\top$ lifting),
  Ch. 4 (body-point Jacobians), Ch. 6 (A\* as the global planner we polish; backward Dijkstra as iLQR's
  discrete cousin), Chs. 12–13 (initial paths), Ch. 17 (`Dynamics`, which iLQR linearizes), Ch. 18 (time
  scaling as the last stage; Pontryagin explains its bang-bang result).
- **Feeds into:** Ch. 21 (§12.5.3–12.5.4 nonlinear optimization and gradient steering import
  `trajopt::optim`; Choset's §12.5.3 cross-links here), Ch. 22 (learned initializations), Ch. 23.
- **Baseline sources:** C §11.3 opening (b. 384–385: the four direct approaches), §11.3.1 (b. 385–389:
  state form 11.24, objective 11.25, Hamiltonian 11.26, minimum principle 11.27, extremality 11.28,
  Legendre–Clebsch 11.29, adjoint 11.30, Example 11.3.1 min-effort double integrator, the min-time
  bang-bang recovery, shooting), §11.3.2 (b. 389–392: the program 11.33–11.39, the three
  parameterization choices, the discretized program 11.40–11.45, penalty variant, SQP, smoothness and
  analytic gradients, local optima and multi-start); C §12.5.3 (b. 445–446) read for the cross-link.
  Classical texts in Choset's bibliography: Bryson & Ho [C ref. 72], Kirk [C ref. 236], Pontryagin et al.
  [C ref. 354]. Sister book: Ch. 23 (`ch23-mppi`, sampling-based MPC — w19.3's comparison partner), Ch. 19
  (`ch19-map-representations`, ESDF), Ch. 21 (`ch21-mdps`, value functions), Ch. 15 (`ch15-factor-graphs`,
  its own GN/LM — cross-linked, not imported). Modernization set, all (verify): Ratliff, Zucker, Bagnell &
  Srinivasa, CHOMP (ICRA 2009), Zucker et al. (IJRR 2013); Kalakrishnan et al., STOMP (ICRA 2011);
  Schulman et al., TrajOpt (RSS 2013; IJRR 2014); Li & Todorov, iLQR (ICINCO 2004); Jacobson & Mayne,
  DDP (1970); Tassa, Erez & Todorov (IROS 2012).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; $D(x)$, $\nabla D$, $J(q)$, $M, C, g$ are book-wide):

| Symbol | Meaning |
|---|---|
| $x = (q, \dot q)$, $\dot x = f(x, u)$ | state and state equation (C eq. 11.24) |
| $J = \int_0^{t_f} \ell(x, u, t)\,dt$ | objective; $\ell$ the running cost (Choset's optimal-control Lagrangian, renamed to avoid the dynamics $L$) |
| $H = \ell + \lambda^\top f$, $\lambda(t)$ | Hamiltonian; costate (Lagrange multipliers) |
| $X$ | finite parameter vector of the transcribed program (C eq. 11.40) |
| $\xi = (q_1, \dots, q_{N-1})$ | discretized trajectory with fixed endpoints $q_0, q_N$ |
| $\mathsf{E}$, $A = \mathsf{E}^\top\mathsf{E}$, $\mathcal{F}_{smooth} = \tfrac12\|\mathsf{E}\xi + e\|^2$ | finite-difference operator; smoothness metric; smoothness functional |
| $c(d)$, $\epsilon$, $\mathcal{F}_{obs}$, $\lambda_{obs}$ | obstacle cost of a distance, its margin, the obstacle functional and its weight |
| $\bar\nabla\mathcal{F} = A^{-1}\nabla\mathcal{F}$, $\eta$ | covariant gradient; CHOMP step-size parameter (step is $1/\eta$) |
| $x_k, u_k$, $f_x, f_u$, $\ell_x, \ell_{xx}, \dots$ | discrete-time trajectory and its local derivatives |
| $V_k(x)$, $V_x, V_{xx}$; $Q_x, Q_u, Q_{xx}, Q_{uu}, Q_{ux}$ | value function and its quadratic model; the action-value expansion |
| $k_k$, $\mathbf{K}_k$, $\mu$, $\alpha$ | feedforward and feedback gains; regularization; line-search step |
| $sd(\cdot,\cdot)$, $\hat n$, $\Delta$, $\mu_{pen}$; $N_h$ | signed distance, contact normal, trust radius, penalty weight (TrajOpt); MPC horizon |

**Definitions:**
- *Direct trajectory planning* (C §11.3): search the full state space for $(x(t), u(t))$ jointly.
- *Extremal control*: a $u^*$ with $\partial H/\partial u = 0$ along the trajectory — necessary, not
  sufficient; *locally optimal* if also $\partial^2 H/\partial u^2 > 0$ (Legendre–Clebsch).
- *Singular optimal control*: extremal with $\partial^2 H/\partial u^2$ only semidefinite — the min-time
  double integrator, and Ch. 18's zero-inertia arcs.
- *Transcription*: a finite $X$ and finitely many constraint checks at $j\Delta t$ (C eqs. 11.40–11.45).
- *Covariant gradient*: steepest descent in the metric $A$, $\bar\nabla\mathcal{F} = A^{-1}\nabla\mathcal{F}$
  — a perturbation measured by how much it changes the *whole* path's smoothness.
- *Receding horizon (MPC)*: each tick, solve a horizon-$N_h$ problem from the measured state, apply the
  first control, discard the rest, repeat.

**Derivations:**

1. **Pontryagin and the min-effort double integrator (C Example 11.3.1).** *Statement:* for $\ddot q
   = u$, $\ell = u^2$, rest-to-rest over distance $d$ in fixed $t_f$, the extremal control is $u(t) =
   6d/t_f^2 - 12d\,t/t_f^3$, and it is a local minimum. *Sketch (5 steps):* $H = u^2 + \lambda_1 x_2 +
   \lambda_2 u$; $\partial H/\partial u = 2u + \lambda_2 = 0$; adjoint $\dot\lambda_1 = 0$,
   $\dot\lambda_2 = -\lambda_1$ so $\lambda_2$ is affine in $t$ and so is $u$; impose $\int_0^{t_f} u =
   0$ and $\int_0^{t_f}\!\!\int_0^t u = d$ to fix $c_0 = 6d/t_f^2$, $c_1 = -12d/t_f^3$; $\partial^2
   H/\partial u^2 = 2 > 0$. *Collapsible:* the integrals, the cubic $q(t)$, and the remark that this is
   the minimum-acceleration-energy profile every spline library rediscovers.
2. **Min-time is bang-bang, by the minimum principle (C §11.3.1).** *Statement:* with $\ell = 1$,
   $|u| \le u_{max}$ and $t_f$ free, $u^* = -u_{max}\,\mathrm{sign}(\lambda_2)$ with at most one
   switch. *Sketch (4 steps):* $H = 1 + \lambda_1 x_2 + \lambda_2 u$ has $\partial H/\partial u =
   \lambda_2$ — no $u$, no convexity; the inequality $H^* \le H$ reduces to $\lambda_2 u^* \le \lambda_2
   u$ for all feasible $u$; so $u^*$ sits at the bound opposite to the sign of $\lambda_2$; $\lambda_2$
   is affine in $t$ and changes sign at most once. *Collapsible:* the terminal conditions fixing the
   switch, the match with Ch. 18's $t_s = \sqrt{d/u_{max}}$, and *shooting* — guess $\lambda(0)$, integrate
   state and adjoint, correct from the terminal miss — where the "initial guess" problem is first explicit.
3. **Transcription (C §11.3.2).** *Statement:* the program 11.33–11.39 becomes the finite program
   11.40–11.45 once (a) $q(t)$, or (b) $u(t)$, or (c) both are parameterized and the inequality
   constraints are enforced at $k+1$ points; (a) computes $u$ from the dynamics, (b) integrates them,
   (c) adds dynamics as equality constraints. *Sketch (3 steps):* choose a basis (polynomial, Fourier,
   spline, piecewise-constant — Choset's list); sample the constraints; hand $J(X)$, constraints, and
   their gradients to an SQP solver. *Collapsible:* why $C^2$ smoothness in $X$ matters; the penalty
   variant; the modern names — (a) state collocation (CHOMP), (b) direct shooting (iLQR), (c) full
   collocation (TrajOpt).
4. **CHOMP's covariant functional gradient (verify).** *Statement:* with $\mathcal{F} = \mathcal{F}_{smooth}
   + \lambda_{obs}\mathcal{F}_{obs}$, $\nabla\mathcal{F}_{smooth} = A\xi + b$ and, for body point $x(q,
   \beta)$ moving with speed $\|x'\|$, $\nabla_{q}\mathcal{F}_{obs} = \sum_\beta J_\beta^\top \|x'\|
   \big[(\mathbb{1} - \hat x'\hat x'^\top)\nabla c - c\,\kappa\big]$; the update $\xi \leftarrow \xi -
   \tfrac1\eta A^{-1}\nabla\mathcal{F}$ is steepest descent in the smoothness metric. *Sketch (5
   steps):* write $\mathcal{F}_{smooth}$ as a quadratic in $\xi$; write $\mathcal{F}_{obs}$ as an arc-length
   integral of $c(D(x))$ over body points (invariant to retiming); differentiate under the integral,
   producing the velocity projection and curvature term; pull workspace gradients to joint space with
   $J_\beta^\top$ (Ch. 7 §4.7); minimize the first-order model subject to $\tfrac12\delta\xi^\top A\,\delta\xi \le$ const. *Collapsible:* $c(d) = (\epsilon - d)^2/(2\epsilon)$
   on $(0, \epsilon)$, $-d + \epsilon/2$ below, $0$ above, with its $C^1$ continuity; why $A^{-1}$
   spreads a local push along the whole path; STOMP's gradient-free replacement: noise $\delta\xi \sim
   \mathcal{N}(0, A^{-1})$, $K$ rollouts, cost-weighted update $\sum_k w_k\delta\xi_k$, $w_k \propto
   \exp(-\mathcal{F}_k/\lambda)$.
5. **iLQR backward pass.** *Statement:* with $Q_x = \ell_x + f_x^\top V'_x$, $Q_u = \ell_u + f_u^\top
   V'_x$, $Q_{xx} = \ell_{xx} + f_x^\top V'_{xx} f_x$, $Q_{uu} = \ell_{uu} + f_u^\top V'_{xx} f_u$,
   $Q_{ux} = \ell_{ux} + f_u^\top V'_{xx} f_x$, the optimal local policy is $\delta u = k + \mathbf{K}
   \delta x$ with $k = -Q_{uu}^{-1}Q_u$, $\mathbf{K} = -Q_{uu}^{-1}Q_{ux}$, and the value model updates
   as $V_x = Q_x - \mathbf{K}^\top Q_{uu} k$, $V_{xx} = Q_{xx} - \mathbf{K}^\top Q_{uu}\mathbf{K}$. *Sketch
   (5 steps):* Bellman's equation $V_k(x) = \min_u[\ell + V_{k+1}(f(x,u))]$; expand to second order
   about the nominal $(x_k, u_k)$; minimize the quadratic in $\delta u$; substitute back; recurse from
   $V_N = \ell_f$. *Collapsible:* dropping $f_{xx}, f_{uu}, f_{ux}$ (iLQR) vs. keeping them (DDP) trades
   quadratic convergence for cheaper iterations; regularization $Q_{uu} + \mu\mathbb{1}$ and Tassa et al.'s
   $V_{xx}$ variant; the forward pass $u_k + \alpha k_k + \mathbf{K}_k(x_k - \bar x_k)$ with backtracking
   on $\alpha$; on a linear-quadratic problem one backward pass *is* the Riccati recursion.
6. **TrajOpt's sequential convex optimization (verify).** *Statement:* minimize $J(X) + \mu_{pen}\sum
   |g_i(X)|^+$ by repeatedly convexifying $J$ and $g$ about the current $X$, solving the convex
   subproblem inside a trust region $\|\delta X\|_\infty \le \Delta$, accepting if the true merit
   improves, and increasing $\mu_{pen}$ in an outer loop until constraints are satisfied; collision
   constraints are $sd(R(q), \mathcal{WO}_i) \ge d_{safe}$, linearized as $sd(q) + \hat n^\top J_{p}(q)\,
   \delta q$ at the witness point $p$. *Sketch (4 steps):* the $\ell_1$ penalty is exact for finite
   $\mu_{pen}$; convexify; the trust region guarantees descent on the merit function; penalty loop. *Collapsible:* why constraints beat costs for collision (zero violation at convergence
   vs. CHOMP's "nearly out"); swept volumes as convex hulls of consecutive footprints (recipe-level);
   what our `sco` simplifies (LM with a box trust region, not a general QP) and says so.

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| Shooting (C §11.3.1) | `(λ₀ guess, f, H) -> x(t), u(t)` | one state+adjoint integration per iteration |
| Transcription + SQP (C §11.3.2) | `(X₀, J, g, h) -> X*` | per iteration one QP in $\dim X$ |
| `chomp` / `stomp` (verify) | `(ξ₀, D, J_β, η, λ_obs, ε, iters[, K, rng]) -> ξ*` | $O(N\,\beta)$ distance queries + one banded $A^{-1}$ solve per iteration; STOMP: $K$ seeded rollouts, no gradients |
| `sco` / TrajOpt (verify) | `(X₀, J, g, Δ, μ_pen) -> X*` | outer penalty loop × inner trust-region LM |
| `ilqr` / `ddp` (verify) | `(x₀, ū, f, ℓ, ℓ_f, N) -> (x̄, ū, K)` | $O(N (n_x + n_u)^3)$ per iteration |
| `mpc_loop` | `(x_meas, solver, N_h) -> u₀` each tick | one warm-started solve per tick |
| `optim::lm` (ours, shared) | `(r, J_r, x₀, trust) -> x*` | GN/LM with trust region; Ch. 21 imports it |

**Numeric micro-example** (the contract with the unit test). A point robot in the plane, three
waypoints with the ends fixed: $q_0 = (0, 0)$, $q_2 = (2, 0)$, free $q_1 = (1, 0.2)$. One disc obstacle
centered at $(1, 0)$ with radius $0.5$, so the distance field is $D(x) = \|x - (1,0)\| - 0.5$ and at
$q_1$, $D = 0.2 - 0.5 = -0.3$ (inside), $\nabla D = (0, 1)$. Margin $\epsilon = 0.5$, $\lambda_{obs} =
1$, $\eta = 1$, unit spacing, and the pointwise obstacle gradient (no velocity weighting — the test pins
exactly this form; the full form is a second test). Smoothness: $\mathcal{F}_{smooth} = \tfrac12(\|q_1 -
q_0\|^2 + \|q_2 - q_1\|^2)$, so $\nabla_{q_1}\mathcal{F}_{smooth} = 2q_1 - q_0 - q_2 = (0, 0.4)$ and
$A = 2\cdot\mathbb{1}$. Obstacle: $D < 0$ so $c = -D + \epsilon/2 = 0.55$ and $c'(D) = -1$, giving
$\nabla_{q_1}\mathcal{F}_{obs} = c'\,\nabla D = (0, -1)$. Total gradient $(0, -0.6)$; covariant step
$q_1 \leftarrow q_1 - \tfrac1\eta A^{-1}(0, -0.6) = (1, 0.2) + (0, 0.3) = (1, 0.5)$: one step lands the
waypoint exactly on the obstacle surface ($D = 0$). Second step: $\nabla\mathcal{F}_{smooth} = (0, 1.0)$,
$c'(0) = -(\epsilon - 0)/\epsilon = -1$ so $\nabla\mathcal{F}_{obs} = (0, -1)$, total $0$ — a fixed point
with zero clearance. In general the equilibrium is $y^* = \lambda_{obs}/(1 + \lambda_{obs})$, clearance
$D^* = (\lambda_{obs} - 1)/(2(1 + \lambda_{obs}))$: $0$ for $\lambda_{obs} = 1$, $1/6$ for $2$, $0.3$ for
$4$. The lesson: CHOMP's obstacle term is a *cost*; clearance is bought with $\lambda_{obs}$ and $\epsilon$.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"a rubber band in a force field."** Colors: initial path **blue** (a roadmap/tree
output), optimized path **purple**, executed trajectory **orange**, obstacles **slate** over the Ch. 7
distance-field colormap, start/goal **green**/**red**. Every widget autoplays and ships an SVG fallback;
the one stochastic mode (STOMP) is seeded.

- **Widget w19.1: CHOMP Smoother** — *flagship, wasm-sim.* Left: Reach on the Workbench with the
  Ch. 7 distance field underlay and body points marked; right: the same path on the unwrapped torus.
  **Manipulates:** headline — step size $1/\eta$ (too large oscillates, too small crawls); disclosure —
  $\lambda_{obs}$, $\epsilon$, $N$, initial path (the Ch. 12 RRT output *or* the other homotopy class
  around the obstacle), **STOMP mode** with $K$ and seed re-roll (`Pcg64`, seed shown). **Observes:**
  the rubber band relaxing in both views; stacked cost bars; a "converged in collision" badge when
  $D < 0$ survives; the clearance converging to $D^*$ on the micro-example preset. **Misconception
  killed:** "optimization finds the best path" — it finds the nearest stationary point of the class it
  started in; switch the initial path and the answer changes.
- **Widget w19.2: iLQR Backward Pass** — *interactive animation.* Hitch's kinematic car on a nominal
  trajectory in the Lot. **Manipulates:** headline — scrub $k$ from $N$ down to $0$: at each step the
  quadratic value model appears as a $V_{xx}$ ellipse around $\bar x_k$ with the feedforward arrow $k_k$
  and the feedback gain $\mathbf{K}_k$ drawn as a small vector field; disclosure — $\mu$, $N$, cost
  weights, DDP on/off, and a **forward pass** button that rolls out with line search and shows $\alpha$.
  **Observes:** ellipses tightening toward the goal; the new nominal after each iteration; cost dropping
  quadratically near convergence. **Misconception killed:** "iLQR is gradient descent on the control
  sequence" — it builds a local value function and a *feedback policy*; the rollout is closed-loop.
- **Widget w19.3: MPC Horizon** — *wasm-sim (added).* Rusty in the Apartment under receding-horizon
  iLQR with a scripted moving obstacle. **Manipulates:** headline — horizon $N_h$; disclosure — tick
  rate, warm start, obstacle speed, open-loop ghost. **Observes:** the horizon tail redrawn each tick; a
  myopic dead-end at short horizons; a solve-time tile growing with $N_h$; the open-loop ghost colliding
  where MPC swerves. **Misconception killed:** "a longer horizon is always better" — past the point where
  the plan is corrected anyway, horizon buys compute, not safety.
- **Figure f19.4: The Optimizer Family Map** — *static-svg with hover.* Axes: first-order / second-order /
  sampling; obstacles as cost vs. constraint; shooting vs. collocation. Nodes: shooting, transcription/SQP
  (C §11.3.2), CHOMP, STOMP, TrajOpt, iLQR/DDP, MPC, MPPI (sister Ch. 23). **Misconception killed:**
  "these are competing algorithms" — they are points in one design space.

Dashboard layout: w19.1 full-width atop the C section, cost bars docked beneath; w19.2 half-width beside
derivation 5 (hovering $Q_{uu}$ or $\mathbf{K}_k$ highlights the ellipse or field); w19.3 full-width in
the MPC subsection; f19.4 closes the chapter.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`SVector`/`SMatrix` states; $A^{-1}$ hand-rolled as a block-tridiagonal
Thomas sweep), `potential` (Ch. 7 — `DistanceField`, grid distance transform), `robots` + `cspace`
(Chs. 2/4 — body-point Jacobians), `dynamics` (Ch. 17), `trajectory` (Ch. 18 — `CubicSpline`,
`time_scale`), `rand_pcg` 0.9 (`Pcg64` for STOMP), `argmin` — **comparison only**: its L-BFGS and
Gauss–Newton must reach the same stationary points as ours in tests; `widget-kit`, `plotters`.

**Module plan:**
- `crates/trajopt/` — **introduced here; imported by Chs. 21, 23.**
  - `src/optim.rs` — `lm` (Gauss–Newton/Levenberg–Marquardt with trust region), `line_search`; the
    shared optimizer Ch. 21 uses for nonlinear steering.
  - `src/chomp.rs` — `ChompProblem`, `chomp_step`, `chomp` (both obstacle-gradient forms).
  - `src/stomp.rs` — seeded rollouts, cost-weighted update; shares `ChompProblem`.
  - `src/sco.rs` — penalty + trust-region sequential convexification; signed distances, witness points
    and normals from `collide` (Ch. 2's `parry2d` black box).
  - `src/ilqr.rs` — `trait DiscreteDynamics`, `QuadCost`, `BackwardPass`, `ilqr` (DDP behind a flag).
  - `src/mpc.rs` — `Mpc` receding-horizon wrapper with warm start.
  - `src/models.rs` — `Unicycle` (Rusty), `KinematicCar` (Hitch, from `robots`), `ReachDiscrete`
    (`dynamics::Reach2R` + RK4 step).
  - `examples/chomp_three.rs`, `examples/ilqr_min_effort.rs`, `tests/`.
- `demos/ch19-demo/` — eframe app hosting w19.1–w19.3.

**Key types & signatures** (compiles in spirit):

```rust
use nalgebra::{SMatrix, SVector, DVector};
use potential::DistanceField;   // Ch. 7: fn dist(&self, x: Point2) -> f64; fn grad(&self, x) -> Vector2

/// A discretized trajectory with fixed endpoints; ξ holds only the N−1 free waypoints.
pub struct ChompProblem<'a, const N: usize> {
    pub q0: SVector<f64, N>, pub q_end: SVector<f64, N>,
    pub field: &'a dyn DistanceField,
    /// Body points: workspace position and Jacobian at q (Reach: samples along both links).
    pub body: Vec<Box<dyn Fn(&SVector<f64, N>) -> (nalgebra::Point2<f64>, SMatrix<f64, 2, N>) + 'a>>,
    pub eps: f64, pub lambda_obs: f64, pub eta: f64,
    /// `Pointwise` is the micro-example's form; `ArcLength` is the full CHOMP gradient.
    pub obstacle_form: ObstacleGradient,
}
pub enum ObstacleGradient { Pointwise, ArcLength }

/// One covariant step ξ ← ξ − (1/η) A⁻¹ ∇F. A is block-tridiagonal, solved in O(N).
pub fn chomp_step<const N: usize>(p: &ChompProblem<'_, N>, xi: &mut [SVector<f64, N>]) -> ChompStats;
// ChompStats: f_smooth, f_obs, min_clearance — the widget's cost bars and badge.

/// Discrete-time dynamics x_{k+1} = f(x_k, u_k) with Jacobians; implemented by `models::*`.
pub trait DiscreteDynamics<const NX: usize, const NU: usize> {
    fn step(&self, x: &SVector<f64, NX>, u: &SVector<f64, NU>) -> SVector<f64, NX>;
    /// (f_x, f_u). Default: central differences — override with analytic forms for speed.
    fn jacobians(&self, x: &SVector<f64, NX>, u: &SVector<f64, NU>)
        -> (SMatrix<f64, NX, NX>, SMatrix<f64, NX, NU>);
}

pub struct BackwardPass<const NX: usize, const NU: usize> {
    pub k: Vec<SVector<f64, NU>>,          // feedforward
    pub big_k: Vec<SMatrix<f64, NU, NX>>,  // feedback gains 𝐊_k — the policy, not just a plan
    pub expected_decrease: (f64, f64),     // ΔV(α) = α·a + α²·b, for the line search
}

pub struct IlqrResult<const NX: usize, const NU: usize> { /* xs, us, gains, per-iteration costs */ }
pub fn ilqr<const NX: usize, const NU: usize>(
    f: &impl DiscreteDynamics<NX, NU>, cost: &QuadCost<NX, NU>,
    x0: SVector<f64, NX>, u_init: &[SVector<f64, NU>], opts: IlqrOptions,
) -> IlqrResult<NX, NU>;

/// Receding horizon around any solver that accepts a warm start.
pub struct Mpc<S> { pub solver: S, pub horizon: usize, pub warm: Option<DVector<f64>> }
```

**Worked end-to-end example:** `cargo run --example chomp_three -p trajopt` builds the micro-example
(`DiscField`, an analytic disc implementing `DistanceField` so the numbers are exact; the widget uses
the Ch. 7 grid field) and prints `grad_smooth = (0.0000, 0.4000)`, `grad_obs = (0.0000, -1.0000)`,
`q1 <- (1.0000, 0.5000)`, `clearance = 0.0000`, then `step 2: |grad| = 0.0000 (fixed point)`, and the
$\lambda_{obs} \in \{1, 2, 4\}$ equilibria `0.0000 0.1667 0.3000` after 200 iterations each.
`#[test] fn reproduces_micro_example()` asserts all to $10^{-4}$; `#[test] fn argmin_agrees()` runs
`argmin`'s L-BFGS on the same objective and asserts the same fixed point to $10^{-6}$.
`cargo run --example ilqr_min_effort -p trajopt` discretizes $\ddot q = u$ by zero-order hold at $N =
100$, $t_f = 1$, $d = 1$, cost $\sum u_k^2\Delta t$ plus terminal weight $10^6\|x_N - (1, 0)\|^2$, and prints
$u_k$ against Choset's $6 - 12t$: `#[test] fn ilqr_matches_pontryagin()` asserts $\max_k |u_k - (6 -
12\,t_{k+1/2})| < 5\cdot 10^{-2}$ and convergence in one iteration (linear-quadratic: one pass *is* Riccati).

**Runnable artifact:** the WASM build of `trajopt` *is* w19.1 (`chomp_step` per frame), w19.2
(`BackwardPass` scrubbed step by step), and w19.3 (`Mpc<Ilqr>` ticking). The Integration lab runs the
Reach pipeline — Ch. 12 `Rrt` → Ch. 13 shortcut → `chomp` → Ch. 18 `CubicSpline` + `time_scale` →
Ch. 17 replay — reporting $t_f$ before and after optimization; and Rusty under `Mpc<Ilqr>` beside a
replay of the sister book's MPPI on the same model and horizon, for an honest side-by-side.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w19.1 | CHOMP Smoother | wasm-sim | trajopt + potential (Ch. 7) + robots/cspace + widget-kit, Pcg64 (STOMP) | step size, $\lambda_{obs}$/$\epsilon$/$N$, initial-path choice, STOMP mode, seed re-roll | covariant descent; cost vs. clearance; local minima by homotopy |
| w19.2 | iLQR Backward Pass | interactive animation | trajopt::ilqr + robots (Hitch) + widget-kit | scrub $k$, $\mu$, $N$, DDP toggle, forward-pass button | local value functions and feedback gains; line search |
| w19.3 | MPC Horizon | wasm-sim | trajopt::mpc + sim (Apartment) + widget-kit | horizon slider, warm start, obstacle script, open-loop ghost | receding horizon; myopia vs. compute |
| f19.4 | The Optimizer Family Map | static-svg (hover) | plotters build-time + CSS hover | hover nodes | one design space, not a zoo |
| — | Before/after hook | animation (autoplay) | trajopt + trajectory + dynamics | replay | optimization makes a path smooth, fast, and feasible — in its homotopy class |

## 7. Exercises & Extensions

1. **(F)** Derive $y^* = \lambda_{obs}/(1 + \lambda_{obs})$ for the three-waypoint micro-example and show
   the equilibrium is stable under the covariant update for every $\eta \ge 1$.
2. **(F)** Show that for linear dynamics and quadratic cost the iLQR backward pass is the discrete
   Riccati recursion and the $\alpha = 1$ forward pass is optimal in one iteration; exhibit the first
   nonlinear term that breaks this.
3. **(C, predict-then-verify)** In w19.1 load the micro-example preset and set $\lambda_{obs} = 4$.
   Predict the converged clearance before pressing run; then set $\epsilon = 1.0$ and predict again.
4. **(C)** In w19.3 find the shortest $N_h$ at which Rusty still rounds the kitchen corner; explain the
   threshold via the Ch. 6 backward-Dijkstra cost-to-go, and verify that adding it as $\ell_f$ lets a
   shorter horizon succeed.
5. **(P)** Implement DDP's second-order terms by finite differences in `ilqr.rs`; compare iteration
   counts and wall-clock time against iLQR on Hitch's parking problem over 20 seeded starts.
6. **(P, stretch)** Implement STOMP's update with `Pcg64`; run 20 seeds against CHOMP on the two-homotopy
   Workbench scene and report success rate and median cost honestly, including the seeds that never
   escape the trap.

## 8. Modernization Notes

- **Kept, in full:** Choset's §11.3.1 — Hamiltonian, minimum principle, both double-integrator
  examples, singular controls, shooting — because every modern method answers the conditions he
  states numerically; §11.3.2's three parameterization choices (the shooting/collocation taxonomy) and
  his two warnings (smoothness; local optima and multi-start), which become widgets; §12.5.3's remark
  that underactuated systems must parameterize $u(t)$, not $q(t)$ — Ch. 21's hook.
- **Added beyond 2005 (all (verify) until authoring):** CHOMP's covariant gradient on a distance field
  and its $J^\top$ pull-back (Choset's own §4.7 lifting — a connection the prose should celebrate);
  STOMP as the seeded gradient-free variant; TrajOpt's $\ell_1$-penalty sequential convex optimization;
  iLQR/DDP with regularization and line search; MPC as the loop; the Ch. 7 distance field and the
  sister's ESDF/MPPI as shared infrastructure; `argmin` as the single comparison library.
- **Dropped/condensed:** the potential-field "fourth approach" (C b. 385) is a pointer to Ch. 7; RRTs as
  the "third approach" point to Chs. 12 and 21; Choset's solver-package list (FSQP, NPSOL, IMSL, NAG,
  MATLAB) becomes one sentence and `argmin`; full SQP is not implemented — `sco` is a labeled recipe.
- **Honesty items the prose must keep:** every method here is *local* — never "optimal" without
  "locally"; CHOMP's obstacle term is a cost, so collisions can survive convergence and the widget shows
  it; `sco` is not TrajOpt's QP-based SQP and says so; iLQR needs $\mu$ and $\alpha$ and the defaults are
  stated; MPC stability (terminal cost as Lyapunov function) is a pointer, not a theorem proved here; the
  MPPI comparison uses the same model, horizon, and seeds or it is not shown.
