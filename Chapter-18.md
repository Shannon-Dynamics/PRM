# Chapter 18 — Trajectory Planning

> Part V — Dynamics, Trajectories, and Constraints · Estimated length: 10 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

A path is a curve; a trajectory is a curve with a clock. Every planner in Parts II–III handed Reach a
curve and walked away. This chapter attaches the clock — and discovers that the best clock is almost
never uniform. The "aha": **once the path is fixed, an $n$-DOF arm with $n$ torque limits becomes a
one-dimensional system in the path parameter $s$, and its time-optimal execution is a curve in the
$(s, \dot s)$ phase plane that hugs a velocity-limit curve and switches between maximum and minimum
acceleration a finite number of times.** The reader who can read that phase plane can read every
time-scaling result since 1985. The second half of the chapter asks what happens when the path is *not*
fixed, and meets the ancestor of every state-lattice planner: Choset's grid search over $(q, \dot q)$,
run on the Ch. 6 A\*.

Story line:
1. **Hook** — the Ch. 17 torque gauge again: a Ch. 12 path for Reach executed at constant $\dot s$ pegs
   a motor; the same path executed by this chapter's time scaler finishes in $0.79$ s and never pegs.
   Same curve, two clocks.
2. **Play (C)** — w18.1 Phase-Plane Racer: drag the path's waypoints, watch the velocity-limit curve and
   the max/min acceleration integral curves redraw, and the switch points snap into place.
3. **Formalize (F)** — path, time scaling, trajectory; substituting $q(s(t))$ into the Ch. 17 equations
   to get $a(s)\ddot s + b(s)\dot s^2 + c(s) = u$; acceleration bounds $L, U$; the motion cone and the
   velocity-limit curve; bang-bang optimality; zero-inertia, critical, and singular points.
4. **Algorithm** — the five-step Time-Scaling Algorithm (Bobrow–Dubowsky–Gibson, Shin–McKay) with its
   zero-inertia modification; the global problem (Shiller–Dubowsky sketch); Choset's GRID SEARCH over
   $(q, \dot q)$ and its completeness theorem; TOPP-RA as the modern reformulation.
5. **Implementation (P)** — the `trajectory` crate: `Path`, `PathDynamics`, `time_scale`,
   `toppra`, and `lattice_search` on the Ch. 6 search crate.
6. **Integration lab** — Ch. 12 RRT path → Ch. 13 shortcutting → spline → time scaling → the Ch. 17
   integrator replays it with the computed torques; the gauge grazes but never crosses its limit.
   **Close:** decoupled planning is only locally honest about time; the direct methods open Ch. 19.

## 2. Prerequisites & Position

- **Builds on:** Ch. 17 (`dynamics`: $M$, $C\dot q$, $g$, `forward`, RK4), Ch. 6 (`search`: A\* for
  the $(q, \dot q)$ lattice), Chs. 11–13 (the paths being time-scaled; shortcutting yields the polyline
  the spline smooths), Ch. 5 ($T^2$ unwrapping so splines on Reach's torus are $C^2$ across the seam).
- **Feeds into:** Ch. 19 (the optimal-control view of bang-bang; the time-scaled trajectory is CHOMP's
  and iLQR's warm start), Ch. 21 (state lattices and hybrid A\* as descendants of GRID SEARCH;
  Kinodynamic RRT\* uses the same $(q, \dot q)$ state), Ch. 23 (Chs. 11–13 → 18 → 19 in one pipeline).
- **Baseline sources:** C §11.1 (b. 374: path, time scaling, trajectory), §11.2 (b. 374–384: the
  substitution eqs. 11.2–11.6, bounds 11.7–11.9, the phase plane and velocity-limit curve 11.10, the
  Time-Scaling Algorithm b. 378; 11.2.1 zero-inertia points b. 378–383 with Example 11.2.1; 11.2.2
  global time-optimal planning b. 384), §11.3.3 (b. 392–400: the double-integrator tree, Algorithm 21
  GRID SEARCH b. 395, Theorem 11.3.2 b. 395–396, the generalization to $M(q)$ with $\mathcal{A}(q,
  \dot q)$ and $\hat{\mathcal{A}}$ b. 396–399). Classical references in Choset's bibliography: Shin &
  McKay [C ref. 385], Bobrow, Dubowsky & Gibson [C ref. 51], Pfeiffer & Johanni [C ref. 348], Slotine &
  Yang [C ref. 388], Shiller & Dubowsky [C ref. 383], Donald & Xavier [C ref. 135]. Sister book: Ch. 20
  (`ch20-motion-planning`, hybrid A\* sketch), Ch. 21 (`ch21-mdps`, a discretized state space searched
  backward — the lattice's cousin), Ch. 23 (`ch23-mppi`). Modernization set: Pham & Pham, TOPP-RA
  (IEEE T-RO 2018) (verify); Pivtoraiko, Knepper & Kelly, state lattices (JFR 2009) (verify).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped):

| Symbol | Meaning |
|---|---|
| $s \in [0,1]$, $s(t)$, $\dot s$, $\ddot s$ | path parameter; time scaling $s : [0, t_f] \to [0,1]$; its rates |
| $q(s)$, $q' = dq/ds$, $q''$ | the path and its derivatives in $s$ (twice differentiable) |
| $a(s), b(s), c(s)$ | inertial, velocity-product, gravity vectors of the path-constrained dynamics |
| $u_i^{min}(s,\dot s) \le u_i \le u_i^{max}(s,\dot s)$ | actuator limits, possibly state dependent |
| $\alpha_i, \beta_i$; $L_i, U_i$; $L(s,\dot s), U(s,\dot s)$ | per-actuator acceleration bounds; their max/min over $i$ |
| $v(s)$, $\dot s^{max}(s)$, $\dot s^{max}_{zip}(s)$ | velocity-limit curve; its zero-inertia generalization |
| $F$, $A_i$, $\mathcal{S} = \{s_1, s_2, \dots\}$ | backward min-accel curve; forward curves; the switch list |
| $(s_{lim}, \dot s_{lim})$, $(s_{tan}, \dot s_{tan})$, $\ddot s^{\pm}_{tangent}$ | penetration point; tangent point; tangent accelerations at a singular point |
| $h$, $\mathcal{A}$, $\mathcal{A}(q,\dot q)$, $\hat{\mathcal{A}}$ | lattice timestep; discretized control set; feasible-acceleration parallelepiped and its conservative subset |
| $\delta_v(c_0, c_1)$, $\epsilon$, $T_{opt}$ | speed-dependent safety margin $c_0 + c_1\|\dot q\|$; approximation parameter; optimal time |
| $x = \dot s^2$ | squared speed — the variable that makes the limits linear (TOPP-RA) |

**Definitions:**
- *Path / time scaling / trajectory* (C §11.1): $q : [0,1] \to \mathcal{Q}$ twice differentiable; $s(t)$
  twice differentiable and monotone ($\dot s > 0$ on $(0, t_f)$); the trajectory is $q(s(t))$.
- *Motion cone*: at $(s, \dot s)$ the set of tangent directions with $\ddot s \in [L, U]$; the state is
  *inadmissible* when $L > U$ (the cone is empty).
- *Velocity-limit curve*: $v(s) = \min_{i,j}\dot s_{ij}(s)$ where $L_i = U_j$ solves for $\dot s_{ij}$;
  the locus where the cone collapses to one vector.
- *Bang-bang time scaling*: $\ddot s \in \{L, U\}$ at all times except on singular arcs.
- *Zero-inertia point*: $a_i(s) = 0$ for some $i$; the $i$-th actuator then bounds $\dot s$ directly.
  *Critical point*: a point of $\dot s^{max}$ set by such a bound, where $L < U$. *Singular point*: a
  critical point from which $U$ forward or $L$ backward penetrates the limit curve immediately.
- *$\delta_v(c_0, c_1)$-safe trajectory*: clearance at least $c_0 + c_1\|\dot q\|$ at every instant.

**Derivations:**

1. **Path-constrained dynamics (C eqs. 11.2–11.6).** *Statement:* along $q(s)$, $M\ddot q + \dot
   q^\top\Gamma\dot q + g = u$ becomes $a(s)\ddot s + b(s)\dot s^2 + c(s) = u$ with $a = M(q)q'$,
   $b = M(q)q'' + q'^\top\Gamma(q)q'$, $c = g(q)$. *Sketch (3 steps):* chain rule $\dot q = q'\dot s$,
   $\ddot q = q''\dot s^2 + q'\ddot s$; substitute into the Ch. 17 form 10.10; group by $\ddot s$,
   $\dot s^2$, and constants — the velocity-product term is quadratic in $\dot s$ because $\Gamma$ is
   bilinear. *Collapsible:* componentwise expansion; the RP arm's $a, b, c$ (C eqs. 11.20–11.22) as a
   check; why $b_i$ can vanish when $a_i$ does not and vice versa (C Problem 11.5).
2. **Acceleration bounds and the limit curve (C 11.8–11.10).** *Statement:* with $\alpha_i = (u_i^{max}
   - b_i\dot s^2 - c_i)/a_i$ and $\beta_i$ likewise with $u_i^{min}$, $U_i = \alpha_i, L_i = \beta_i$
   if $a_i > 0$ and swapped if $a_i < 0$; $L = \max_i L_i$, $U = \min_i U_i$; the feasible set is
   $L \le \ddot s \le U$, empty above $v(s)$. *Sketch (4 steps):* isolate $\ddot s$ in each row of
   11.7; dividing by a negative $a_i$ flips the inequality; intersect intervals; the boundary of the
   admissible region is where the intersection degenerates. *Collapsible:* why $L, U, v$ are generally
   non-smooth (max/min of smooth functions); the "at most one admissible→inadmissible switch per $s$"
   assumption and the inadmissible islands it excludes (C footnote 2, Shin & McKay's extension).
3. **Time optimality is bang-bang.** *Statement:* $t_f = \int_0^1 ds/\dot s$, so the time-optimal
   scaling maximizes $\dot s$ pointwise subject to the cones, and its phase-plane curve follows the
   cone boundary ($\ddot s = U$ or $L$) except possibly at zero-inertia points; at least one actuator is
   saturated at all times. *Sketch (4 steps):* any admissible curve below the optimal one takes longer
   (monotonicity of $1/\dot s$); if the curve's tangent were strictly inside the cone we could raise
   $\dot s$ locally without leaving admissibility; the set of all cone-tangent curves from a state is
   bounded by its $U$- and $L$-integral curves (C figure 11.2), so the optimum is a concatenation of
   them; switches are where they meet or where the limit curve forces a change. *Collapsible:* the
   Pontryagin reading (Ch. 19 derives it: $\partial H/\partial u$ contains no $u$, so $H$ is minimized
   at a bound), and the doomed-state argument of C figure 11.2.
4. **Zero-inertia points and singular arcs (C §11.2.1).** *Statement:* where $a_i = 0$, constraint
   $i$ becomes $u_i^{min} \le b_i\dot s^2 + c_i \le u_i^{max}$, a direct bound $\dot s^{max}_{zip}$;
   the true limit is $\dot s^{max} = \min(v, \dot s^{max}_{zip})$; at a singular point the feasible
   accelerations are $\ddot s_{max} = \min(\ddot s^{+}_{tangent}, U)$, $\ddot s_{min} = \max(\ddot
   s^{-}_{tangent}, L)$ with $\ddot s_{tangent} = \dot s^{max}\, d\dot s^{max}/ds$. *Sketch (4
   steps):* the $i$-th row loses its $\ddot s$ term; solving for $\dot s$ gives a velocity bound; at a
   critical point $L < U$ but the cone may point out of the admissible region; clipping to the limit
   curve's tangent keeps the trajectory on it — "sliding" along a singular arc. *Collapsible:*
   left/right tangents at corners (C figures 11.4–11.5); the RP arm's zero-inertia point at $s = 1/2$.
5. **GRID SEARCH is time-optimal for its discretization, and complete in the $\epsilon$ sense (C Alg.
   21, Theorem 11.3.2).** *Statement:* integrating each control in $\mathcal{A} = \{-a_{max}, 0,
   a_{max}\}^{n}$ for time $h$ moves $\dot q_i$ by multiples of $a_{max}h$ and $q_i$ by multiples of
   $\tfrac12 a_{max}h^2$, so all reachable states lie on a lattice; breadth-first expansion by level
   finds the fewest-steps trajectory; for a point robot with $n \in \{2,3\}$ among polyhedral obstacles,
   if a $\delta_v(c_0,c_1)$-safe trajectory of duration $T_{opt}$ exists then, with $h \le v_{max}/
   a_{max}$ and $h \le c_0\epsilon / (2a_{max}c_1(1-\epsilon) + 5v_{max})$, the search returns a $(1 -
   \epsilon)\delta_v$-safe trajectory of duration $\le T_{opt}$ from an $O(\epsilon)$-perturbed start
   to an $O(\epsilon)$ goal neighborhood, in time $O((1/\epsilon)^{3n})$. *Sketch (4 steps):* lattice
   closure under the control set; level = elapsed time, so BFS is uniform-cost search; the safety
   margin absorbs the discretization error of tracking any continuous trajectory by lattice moves;
   the grid size bounds the running time. *Collapsible:* Choset's statement verbatim (proof deferred
   to Donald & Xavier [C ref. 135]); the manipulator generalization — the feasible-acceleration
   parallelepiped $\mathcal{A}(q,\dot q)$, its one-step conservative subset $\hat{\mathcal{A}}$, and
   the eigenvalue bound on $M$ that keeps it from collapsing.
6. **TOPP-RA: reachability on squared speed (verify).** *Statement:* with $x = \dot s^2$ and $u = \ddot
   s$, $dx/ds = 2u$, and each limit $u_i^{min} \le a_i u + b_i x + c_i \le u_i^{max}$ is *linear* in
   $(x, u)$ at fixed $s$; on a grid $s_0 < \dots < s_N$ with $x_{k+1} = x_k + 2u_k\Delta_k$ the set of
   $x_k$ from which some feasible $x_{k+1}$ in a target interval is reachable is itself an interval
   computable by two tiny linear programs; a backward pass yields the controllable intervals, a forward
   greedy pass picks the largest feasible $x$, and the result is time-optimal for the discretization
   with $O(N)$ LPs. *Sketch (4 steps):* the substitution; convexity of the per-stage constraint set;
   interval propagation; greedy forward is optimal because larger $x$ never hurts controllability.
   *Collapsible:* why this sidesteps switch-point and tangent-point root-finding; attribution to Pham &
   Pham (2018), to be verified at authoring.

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| Time-Scaling Algorithm (C §11.2, steps 1–5) | `(a, b, c, limits, ṡ₀, ṡ_f) -> (S, t_f)` | $O(N_{int})$ integration steps per curve; switches $\ll N$ |
| Zero-inertia modification (C §11.2.1) | same, with `ṡ_max` and `s̈_min/max` at singular points | adds root-finding for critical/tangent points |
| Shiller–Dubowsky global search (C §11.2.2) | `(grid, start, goal) -> best path + scaling` | branch-and-bound over grid paths; sketch only |
| C Alg. 21 GRID SEARCH | `((q₀,q̇₀), G, 𝒜, h) -> trajectory ∪ FAILURE` | $O((1/\epsilon)^{3n})$; $3^n$ branching |
| `lattice_search` (ours) | same inputs, A\* with per-axis bang-bang heuristic | same worst case; fewer expansions |
| TOPP-RA (verify) | `(a, b, c, limits, grid) -> x_k, t_f` | $O(N)$ two-variable LPs |

**Numeric micro-example** (the contract with the unit test). Reach from the Ch. 17 micro-example
($L_i = 1$, $r_i = 0.5$, $m_1 = 2$, $m_2 = 1$, $I_1 = 1/6$, $I_2 = 1/12$) on a *horizontal* Workbench
($a_g = 0$), joint 2 held at $\pi/2$ by its own motor while joint 1 sweeps a quarter turn:
$q(s) = (\pi s/2,\ \pi/2)$, so $q' = (\pi/2, 0)$, $q'' = 0$, and $M$ is constant along the path.
Limits $|\tau_1| \le 20$ N·m, $|\tau_2| \le 12$ N·m. Then $a(s) = M q' = (\pi,\ \pi/6) = (3.1416,\
0.5236)$, $b(s) = (\Gamma^1_{11}, \Gamma^2_{11})(\pi/2)^2 = (0,\ 0.5 \cdot 2.4674) = (0,\ 1.2337)$,
$c(s) = 0$. Joint 1: $U_1 = 20/\pi = 6.366$, $L_1 = -6.366$, independent of $\dot s$. Joint 2: $U_2 =
(12 - 1.2337\dot s^2)/0.5236 = 22.918 - 2.3562\dot s^2$, $L_2 = -22.918 - 2.3562\dot s^2$. The only
pair that can collapse is $L_1 = U_2$: $-6.366 = 22.918 - 2.3562\dot s^2 \Rightarrow \dot s^2 = 12.429$,
so **$v(s) = 3.5254$** for every $s$ (joint 1 speed $5.54$ rad/s): faster than this, the elbow motor
cannot supply the centrifugal torque $h\dot q_1^2$ while the shoulder brakes at full torque. Bang-bang:
$\ddot s = +6.366$ until the switch, then $-6.366$; by symmetry the switch is at $s = 1/2$ with
**$t_s = \sqrt{2 \cdot 0.5 / 6.366} = 0.3963$ s**, $\dot s(t_s) = 2.5231 < v$ (the limit curve is never
touched — Choset's Example 11.2.1 situation, one switch), $t_f = 0.7927$ s. Check that joint 2 is
honest at the peak: $\tau_2 = 0.5236 \cdot 6.366 + 1.2337 \cdot 6.366 = 11.19 < 12$ N·m during
acceleration. Gravity on ($a_g = 9.81$) at $s = 1/2$: $c = (10.405, -3.468)$, $U_1 = 3.054$, $L_1 =
-9.678$, and $v(1/2) = 4.080$; at $s = 0$, $c_1 = 19.62$ so $U_1 = 0.121$ — the arm can barely lift.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"the path is the road; the phase plane is a speed-limit sign that changes every
meter."** Colors: path **purple**, executed trajectory **orange**, inadmissible region hatched **amber**
(a C-obstacle in $(s, \dot s)$), lattice explored set **blue**, start/goal **green**/**red**. Every
widget autoplays and ships an SVG fallback.

- **Widget w18.1: Phase-Plane Racer** — *flagship, wasm-sim.* Left: Reach on the Workbench with a
  spline path through draggable waypoints. Right: the $(s, \dot s)$ plane with the inadmissible region,
  $v(s)$, the cones as little fans, $F$ and the $A_i$ curves drawing themselves, switch points as
  ticks. **Manipulates:** headline — drag a waypoint (the whole right panel redraws live); disclosure —
  torque limits per joint, gravity on/off, $\dot s_0, \dot s_f$, a "constant speed" ghost at a chosen
  $\dot s$; seed re-roll draws random waypoints (`SmallRng`, seed shown). **Observes:** the optimal
  curve, $t_f$, the switch list; the ghost's constant-speed line crossing into amber when it is too
  fast; zero-inertia points flagged where some $a_i(s)$ changes sign. **Misconception killed:** "go as
  fast as the slowest joint allows" — the limit is a *coupled* function of configuration and speed,
  and the admissible region can have a narrow waist a greedy controller cannot pass.
- **Widget w18.2: Lattice Hopper** — *wasm-sim.* The $(q, \dot q)$ lattice for a 1-DOF (then 2-DOF)
  double integrator with obstacles in $q$. **Manipulates:** headline — $h$ (the grid coarsens and
  the branching tree shortens); disclosure — $a_{max}$, $v_{max}$, safety $c_0, c_1$, BFS vs. A\*
  with the bang-bang heuristic, goal radius. **Observes:** the level-by-level expansion (blue), pruned
  nodes flashing where $\delta_v$-safety fails, the returned piecewise-constant acceleration, and a
  comparison line: the lattice $t_f$ vs. the continuous bang-bang $t_f$. **Misconception killed:**
  "discretize $q$ and plan in it" — time-optimality lives in the *state* space, and a grid in $q$
  alone cannot represent momentum.
- **Widget w18.3: Saturation Scope** — *interactive animation (added).* Torque-vs-time strips for both
  joints under the w18.1 solution, with the limits as rails. **Manipulates:** headline — scrub time;
  disclosure — overlay the constant-speed execution. **Observes:** at every instant one strip touches
  its rail; the switch time is where the saturated joint changes; on singular arcs *neither* touches.
  **Misconception killed:** "time-optimal means every motor at full power" — exactly one is, in turn.

Dashboard layout: w18.1 full-width atop the C section, its phase plane sticky through derivations 1–4
(hovering $L_i$ or $U_i$ highlights that cone edge); w18.3 docked under it; w18.2 full-width in §11.3.3.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`SVector` paths and $a, b, c$), `dynamics` (Ch. 17 — the only source of
$M, \Gamma, g$), `search` (Ch. 6 — `astar` over an implicit graph), `manifold` (Ch. 5 — `T2` unwrapping),
`rand` 0.9 (`SmallRng` waypoints), `widget-kit`, `plotters`. The time-scaling integrator and the
two-variable LPs are hand-rolled; no optimization crate here.

**Module plan:**
- `crates/trajectory/` — **introduced here; imported by Chs. 19, 21, 23.**
  - `src/path.rs` — `trait Path<const N>`: `q(s)`, `dq(s)`, `ddq(s)`; `CubicSpline<N>` (natural cubic
    through waypoints, $C^2$, unwrapped on $T^2$), `Segment<N>` (straight line; `ddq = 0`).
  - `src/constrained.rs` — `PathDynamics<N>`: computes $(a, b, c)(s)$ from `Dynamics` + `Path`.
  - `src/limits.rs` — `TorqueLimits<N>`, `accel_bounds(s, ṡ) -> (L, U)`, `velocity_limit(s)`,
    `zero_inertia_points()`, optional `VelocityLimits<N>` ($|\dot q_i| \le v_{max,i}$) as extra rows.
  - `src/time_scale.rs` — the five-step algorithm with the §11.2.1 modification; RK4 in $(s, \dot s)$
    with event detection for crossings; `TimeScaling { switches, profile, t_f }`.
  - `src/toppra.rs` — reachability variant on a uniform $s$-grid; closed-form two-variable LPs.
  - `src/lattice.rs` — `LatticeState<N>` with integer coordinates, `lattice_search` on `search::astar`.
  - `examples/reach_sweep.rs`, `tests/micro_example.rs`, `tests/lattice.rs`.
- `demos/ch18-demo/` — eframe app hosting w18.1–w18.3.

**Key types & signatures** (compiles in spirit):

```rust
use nalgebra::SVector;
use dynamics::Dynamics;

/// A twice-differentiable curve q : [0,1] → Q. `ddq` must be continuous: the
/// velocity-product term b(s) needs q'' and a C¹ spline would make it jump.
pub trait Path<const N: usize> {
    fn q(&self, s: f64) -> SVector<f64, N>;
    fn dq(&self, s: f64) -> SVector<f64, N>;
    fn ddq(&self, s: f64) -> SVector<f64, N>;
}

/// a(s) s̈ + b(s) ṡ² + c(s) = u — the Ch. 17 model restricted to a path (C eq. 11.6).
pub struct PathDynamics<'a, D: Dynamics<N>, P: Path<N>, const N: usize> { pub dynamics: &'a D, pub path: &'a P }
impl<D: Dynamics<N>, P: Path<N>, const N: usize> PathDynamics<'_, D, P, N> {
    /// b uses the velocity-product vector at q̇ = q'(s): q'ᵀ Γ q' = C(q, q') q' exactly.
    pub fn abc(&self, s: f64) -> (SVector<f64, N>, SVector<f64, N>, SVector<f64, N>);
}

pub struct TorqueLimits<const N: usize> { pub min: SVector<f64, N>, pub max: SVector<f64, N> }

/// (L, U) at a phase-plane state; `None` component when a_i(s) ≈ 0 (zero-inertia row).
pub fn accel_bounds<const N: usize>(abc: &(SVector<f64, N>, SVector<f64, N>, SVector<f64, N>),
                                     lim: &TorqueLimits<N>, sdot: f64) -> (f64, f64);
/// ṡ_max(s) = min(v(s), ṡ_max_zip(s)); +∞ if no pair of rows can collapse.
pub fn velocity_limit<const N: usize>(abc: &(SVector<f64, N>, SVector<f64, N>, SVector<f64, N>),
                                       lim: &TorqueLimits<N>) -> f64;

pub struct TimeScaling {
    pub switches: Vec<f64>,         // s values, in order (the list 𝒮 of C §11.2)
    pub profile: Vec<(f64, f64)>,   // (s, ṡ) samples of the optimal phase-plane curve
    pub t_f: f64,
}
pub enum TimeScaleError { StallsAtZero { s: f64 }, LimitIsland, NoSolution }

/// A time-parameterized curve in configuration space — C §1.3's distinction made a type.
/// Owned by this chapter and imported by Chs. 19, 21, 23: trackers accept a `Trajectory`, never a
/// `Path`, so handing a geometric curve to a controller is a compile error, not a runtime surprise.
pub struct Trajectory<M: Manifold> { pub samples: Vec<(f64 /* t */, M)> }
impl TimeScaling {
    /// Compose the phase-plane profile with the path: (s(t), ṡ(t)) ↦ q(s(t)) sampled every `dt`.
    pub fn into_trajectory<M: Manifold>(&self, path: &impl Path<M>, dt: f64) -> Trajectory<M>;
}

/// Steps 1–5 of C §11.2 with the §11.2.1 singular-point modification.
pub fn time_scale<const N: usize>(
    pd: &PathDynamics<'_, impl Dynamics<N>, impl Path<N>, N>, lim: &TorqueLimits<N>,
    sdot0: f64, sdot_f: f64, ds: f64,
) -> Result<TimeScaling, TimeScaleError>;

/// TOPP-RA (verify): controllable intervals in x = ṡ² backward, greedy forward.
pub fn toppra<const N: usize>(pd: &PathDynamics<'_, impl Dynamics<N>, impl Path<N>, N>,
                              lim: &TorqueLimits<N>, grid: &[f64]) -> Result<TimeScaling, TimeScaleError>;

/// C Alg. 21 on integer lattice coordinates; `search::astar` with h = max_i (per-axis bang-bang time).
pub fn lattice_search<const N: usize>(
    start: (SVector<f64, N>, SVector<f64, N>), goal: (SVector<f64, N>, SVector<f64, N>),
    a_max: f64, v_max: f64, h: f64, safe: &impl Fn(&SVector<f64, N>, &SVector<f64, N>) -> bool,
) -> Option<Vec<SVector<f64, N>>>;   // piecewise-constant accelerations, one per level
```

**Worked end-to-end example:** `cargo run --example reach_sweep -p trajectory` builds the micro-
example and prints `a(0.5) = [3.1416, 0.5236]`, `b(0.5) = [0.0000, 1.2337]`, `v(s) = 3.5254`,
`switches = [0.5000]`, `t_s = 0.3963 s`, `t_f = 0.7927 s`, `peak ṡ = 2.5231`, then with gravity on
`v(0.5) = 4.0799`, `U_1(0) = 0.1210`. `#[test] fn reproduces_micro_example()` asserts each to $10^{-3}$
(the integrator's `ds` sets the tolerance; the test also asserts $|t_f^{\text{TOPP-RA}} - t_f| <
5\cdot 10^{-3}$ on a 200-point grid). `#[test] fn lattice_finds_bang_bang()` runs `lattice_search`
for the 1-DOF double integrator, $a_{max} = 1$, $h = 0.5$, from $(0,0)$ to $(1,0)$: it returns the
four-level control sequence $(+1, +1, -1, -1)$ — velocities $0.5, 1.0, 0.5, 0$ and positions
$0.125, 0.5, 0.875, 1.0$ — matching the continuous bang-bang time $2\sqrt{d/a_{max}} = 2$ s exactly,
with 81 control sequences collapsing to 49 distinct lattice states at level 4.

**Runnable artifact:** the WASM build of `trajectory` + `dynamics` *is* w18.1 (every redraw calls
`time_scale`) and w18.2 (`lattice_search`). The Integration lab chains Ch. 12 `Rrt` → Ch. 13 shortcut
→ `CubicSpline` → `time_scale` → the Ch. 17 RK4 replaying $u(t) = a\ddot s + b\dot s^2 + c$ on
`Reach2R`; the test `replay_tracks_path` asserts the integrated $q(t)$ stays within $10^{-3}$ rad of
$q(s(t))$ and the torques within limits at every sample.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w18.1 | Phase-Plane Racer | wasm-sim | trajectory + dynamics + widget-kit, SmallRng | drag waypoints, torque limits, gravity, boundary speeds, constant-speed ghost, seed re-roll | the velocity-limit curve; switches; coupling between joints |
| w18.2 | Lattice Hopper | wasm-sim | trajectory::lattice + search (Ch. 6) + widget-kit | $h$ slider, $a_{max}/v_{max}$, safety $c_0, c_1$, BFS/A\* toggle | planning in $(q,\dot q)$; discretization vs. optimality |
| w18.3 | Saturation Scope | interactive animation | trajectory + widget-kit | time scrub, constant-speed overlay | one actuator saturated at a time; singular arcs |
| — | Two-clocks hook | animation (autoplay) | trajectory + dynamics + sim | replay | the same path, feasible or infeasible depending on the clock |

## 7. Exercises & Extensions

1. **(F)** Show that one actuator with constant symmetric bounds and $a(s) \ne 0$ has $L < U$ everywhere
   (no velocity-limit curve); then show how a DC-motor bound $u^{max} = u_0 - k\dot s$ creates one.
2. **(F)** For the micro-example path with gravity on, derive $c(s)$ in closed form and show that
   $U_1(0) = (20 - 19.62)/\pi$. What torque limit on joint 1 would make $s = 0$ inadmissible, and what
   does that mean physically?
3. **(C, predict-then-verify)** In w18.1 (gravity off) lower $\tau_2^{max}$ from 12 to 10 N·m. Predict:
   does the switch move before or after $s = 1/2$, and does the trajectory now touch $v(s)$? (Hint:
   $U_2 < U_1$ once $\dot s > 2.325$, while $L$ is unchanged.) Then verify, and read the new $t_f$.
4. **(C)** In w18.2 set $a_{max} = 1$, $d = 1$; predict the level count and lattice $t_f$ at $h = 0.25$;
   then set $c_1 > 0$ near an obstacle and explain the longer, slower detour (C figure 11.11).
5. **(P)** Add joint-velocity limits $|\dot q_i| \le v_{max,i}$ to `limits.rs` as additional rows of
   the velocity-limit computation and show in a test that $v(s)$ becomes $\min(\cdot, v_{max,i}/|q'_i|)$.
6. **(P, stretch)** Implement `toppra` for the Ch. 17 `Reach3R` on a random spline (seeded) and compare
   $t_f$ and wall-clock time against `time_scale` over 50 paths; report where the two disagree by more
   than the grid resolution and explain each case (zero-inertia point? island?).

## 8. Modernization Notes

- **Kept, in full:** Choset's phase-plane presentation (§11.2) — the cones, the inadmissible region,
  the five-step algorithm, and the zero-inertia analysis (§11.2.1), which most modern treatments skip
  and which still bites on real arms; the decoupled-vs-direct framing of the chapter opening; Algorithm
  21 with Theorem 11.3.2 stated precisely and attributed to Donald & Xavier — the honest ancestor of
  every lattice planner, whose $O((1/\epsilon)^{3n})$ bound is why nobody runs it beyond three DOF.
- **Added beyond 2005:** TOPP-RA (Pham & Pham 2018, (verify)) as the reformulation that replaces
  switch-point hunting by interval propagation on $x = \dot s^2$; A\* with an admissible per-axis
  bang-bang heuristic in place of Choset's BFS (same optimality, fewer expansions — a direct use of
  Ch. 6); the explicit lineage GRID SEARCH → state lattices (Pivtoraiko, Knepper & Kelly 2009,
  (verify)) → hybrid A\* (Dolgov et al. 2010, (verify)), developed in Ch. 21; joint-velocity limits as
  first-class constraints; the replay test that closes the loop between planned torques and the Ch. 17
  integrator.
- **Dropped/condensed:** the RP-arm Example 11.2.1 replaced by Reach (its $0.888$ s result survives as
  a cross-check in `tests/rp_arm.rs`, cited as Choset's reported value); Shiller–Dubowsky (§11.2.2) is a
  one-paragraph sketch — modern practice uses Ch. 19's optimizers instead; §11.3.1–11.3.2 go to Ch. 19.
- **Honesty items the prose must keep:** the integration algorithm assumes no inadmissible islands (C
  footnote 2) and the text says what breaks when there are; the lattice is time-optimal *for its
  discretization* and can differ wildly from the true optimum (C b. 396); Theorem 11.3.2's proof is
  beyond the book, as it is beyond Choset; TOPP-RA details are verified against the paper before prose.
