# Chapter 21 — Nonholonomic Systems II: Steering Cars, Trailers, and Underactuated Systems

> Part V — Dynamics, Trajectories, and Constraints · Estimated length: 14 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Chapter 20 proved that Hitch can reach every pose in the Lot. It did not say *how*, and the proof's
own recipe — nested four-flow loops — is a terrible plan: $O(1/\epsilon^2)$ reversals to move
$\epsilon$ sideways. This chapter is the catalog of better answers, and its "aha" is that **every
steering method is a different way of spending the controllability you already own**: solve an
optimal-control problem exactly (Dubins, Reeds–Shepp), change coordinates until the brackets become
a ladder you climb with sinusoids (chained forms), push on the end-state map's Jacobian (gradient
steering), find two coordinates from which the whole state is read off by differentiation
(flatness), or let a search integrate the dynamics forward (Barraquand–Latombe's CAR GRID SEARCH
and the kinodynamic trees descending from it). It closes with Reach's dead third joint driven like a
car along decoupling vector fields, and with the descendants of Choset's grid search inside today's
autonomous vehicles: state lattices, hybrid A\*, Kinodynamic RRT\*. Hitch parks the trailer six ways.

Story line:
1. **Hook** — autoplay: three Hitches park into one bay by Reeds–Shepp primitives (crisp, cusps), a
   flat-output polynomial (smooth, wide), and forward-propagated search (jittery, collision-aware),
   with a scoreboard of length, cusps, planning time, clearance.
2. **Play (C)** — w21.1 Trailer Parker: drag Hitch and the trailer into a bay; switch methods.
3. **Formalize (F)** — exact answers without obstacles (Brockett's sinusoids; chained forms and the
   $(a/4\pi)^k b/k!$ ladder; Dubins and Reeds–Shepp with $LSL$ derived, w21.4), then approximate ones
   (gradient steering on the end-state map; flatness of car and trailers, w21.2).
4. **Algorithms with obstacles** — C Alg. 22 CAR GRID SEARCH; the omnidirectional-to-nonholonomic
   path transformation; PRM/RRT with Reeds–Shepp as `Steer`; §7.5.1's forward propagation (w21.5).
5. **Kinematic reductions (F/P)** — decoupling vector fields, STLKC; Reach's 3R with an unactuated
   third joint driven by the car planner, then time-scaled by Ch. 18 (w21.6). **Modern descendants
   (verify)** — lattices and hybrid A\* as Alg. 22 plus a lattice and analytic Reeds–Shepp shots;
   Kinodynamic RRT\* as RRT\* with optimal-control steering (w21.3).
6. **Implementation (P) and Integration lab** — `nonholonomic::steer`, `nonholonomic::lattice`,
   `kinodynamic`; tests pin one Dubins word and one flat-output point; Hitch parks the trailer among
   the Lot's pillars with every planner; the scoreboard is a test.

## 2. Prerequisites & Position

- **Builds on:** Ch. 20 (STLC verdicts, `symmetric_product`, `lewis_murray`), Ch. 11 (`trait
  Steer`, sampler, kd-tree with the Ch. 5 $SE(2)$ metric — this chapter adds three `Steer` impls and
  no sampler), Ch. 12 (`Rrt`, `extend`), Ch. 13 (`RrtStar`'s rewire), Ch. 6 (A\*), Ch. 14 (w14.3
  preview), Ch. 17 (`dynamics::Model`), Ch. 18 (`time_scale`), Ch. 19 (§12.5.3 is a cross-link, not
  a re-derivation), Ch. 4 ($SE(2)$ slices for arc–segment collision).
- **Feeds into:** Ch. 23 (Hitch parks the trailer; the jackknife failure tour reuses the trailer
  flat-output planner and the lattice), Ch. 22 (learned heuristics for hybrid A\* in the Lot).
- **Baseline sources:** C §12.4.2 (b. 434–437: Theorems 12.4.5, 12.4.7, STLKC); §12.5.1 (b. 440–443);
  §12.5.2 (b. 444–445, eq. 12.31); §12.5.3 (b. 445); §12.5.4 (b. 446–447); §12.5.5 (b. 447–450,
  Example 12.5.1); §12.5.6 (b. 450–461: eq. 12.34–12.38, the Reeds–Shepp set, Alg. 22, path
  transformation, randomized methods, trailers); §12.5.7 (b. 462–465, Table 12.1, Fig. 12.33); §12.5.8
  (b. 465–467); §7.5.1 (b. 253–254); Problems 11–12, 17–18, 20, 23, 25. Classical references at their
  point of use in §3, all from
  Choset's bibliography [C refs. 41, 66, 79, 103, 152, 153, 195, 266, 267, 270–272, 283, 309, 332,
  360, 378, 393, 394, 395, 402, 35]. Sister book: Ch. 20 (`…/chapters/ch20-motion-planning`, whose
  Dubins Dial and hybrid-A\* sketch this chapter completes), Ch. 23 (`…/chapters/ch23-mppi`).
  Modernization set: Dubins 1957 (verify); Dolgov, Thrun, Montemerlo & Diebel, hybrid A\* (verify);
  Pivtoraiko, Knepper & Kelly, state lattices (verify); Webb & van den Berg, Kinodynamic RRT\* (verify).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; Ch. 20's symbols carry over):

| Symbol | Meaning |
|---|---|
| $\rho = r_{min} = L/\tan\gamma$; $v, \omega$ | minimum turning radius; inputs of the reduced car (C eq. 12.35), $|\omega| \le |v|/\rho$ |
| $L_\pm, R_\pm, S_\pm$; $C, S, \mid, C_a, C_{\pi/2}$; $(t, p, q)$ | CAR GRID SEARCH actions (full left / right / straight, forward / reverse); Reeds–Shepp alphabet (arc, straight, cusp, arc of angle $a$, quarter arc); normalized word segments |
| $z = A(x)x$, $v = B(x)u$; $u^p$, $f(x_{start}, u^p)$, $e$; $y$, $\varphi, \psi$ | chained-form coordinates and inputs; control parameterization, end-state map, error; flat outputs and the maps $x = \varphi(y, \dot y, \dots)$, $u = \psi(y, \dot y, \dots)$ |
| $V_i(q)$, $\mathcal{V}$, $w$; $\theta_i$, $d_i$ | decoupling vector fields, their set, kinematic inputs; $i$-th trailer angle, hitch length |
| $G(q_{goal})$, $\mathrm{prop}(x, u, \Delta t)$ | goal region of Alg. 22; forward propagation of $\dot x = f(x, u)$ |

**Definitions:**
- *Steering problem*: $x_{start} \to x_{goal}$ without obstacles, respecting the control system — a
  PRM/RRT *local planner*. *Shape / fiber variables*: coordinates driven directly by the inputs vs.
  coordinates that change only through closed loops in the shape variables.
- *Chained form*: $\dot x = u_1g_1 + u_2g_2$, $g_1 = [1, 0, x_2, x_3, \dots, x_{n-1}]^\top$, $g_2 = e_2$.
  *Singular control / generic loop*: a $u^p$ where $\partial f/\partial u^p$ loses rank; a random
  zero-net-motion control appended to escape it. *Differentially flat system*: $m$ outputs $y_i$ from
  which $x$ and $u$ are functions of $y$ and finitely many derivatives. *Topological property*: for
  every $B_\delta(q)$ there is $B_\epsilon(q)$ whose local paths from $q$ stay in $B_\delta(q)$ (C Fig. 12.28).
- *Kinematic reduction / maximally reducible / decoupling vector field / STLKC*: a driftless
  $\dot q = \sum V_iw_i$ all of whose trajectories are feasible for the mechanical system; one that
  also captures every mechanical trajectory starting in $\mathrm{span}(\mathcal{Y})$; a rank-1
  reduction; $\mathrm{Lie}(\mathcal{V})(q) = T_q\mathcal{Q}$ for some decoupling set. *Forward-propagation
  planner*: a tree planner whose only model access is $\mathrm{prop}$; it steers *toward* a sample,
  never *to* it (C §7.5.1).

**Derivations / theorems:**

1. **Brockett's energy-optimal steering is sinusoidal** (C §12.5.1 [C ref. 66]). *Statement:* for
   $\dot x = [u_1, u_2, x_2u_1]^\top$ and $J = \tfrac12\int_0^1 u^\top u\,dt$, optimal controls are
   quadrature sinusoids of frequency $\lambda_3$; for a pure fiber motion ($x_1, x_2$ return)
   $\lambda_3 = \mp2\pi$ and $x_3(1) = \pm(u_1(0)^2 + u_2(0)^2)/4\pi$. *Sketch (5):* $H = \tfrac12
   u^\top u + \lambda_1u_1 + \lambda_2u_2 + \lambda_3x_2u_1$; $\partial H/\partial u = 0$; adjoint gives
   $\lambda_1, \lambda_3$ constant, $\dot\lambda_2 = -\lambda_3u_1$; hence $\dot u_1 = -\lambda_3u_2$, $\dot u_2
   = \lambda_3u_1$; $\lambda_3 \in 2\pi\mathbb{Z}$ zeroes $\int u_i$. *Collapsible:* the unicycle-to-(12.23) map.
2. **Chained-form systems are STLC and steerable by sinusoids** (C §12.5.2 [C ref. 332]).
   *Statement:* $[g_1, [g_1, \dots [g_1, g_2]]]$ with $k$ copies of $g_1$ equals $(-1)^ke_{k+2}$ (LARC);
   with $u_1 = a\sin 2\pi t$, $u_2 = b\cos 2k\pi t$ over $t_f = 1$, $x_1..x_{k+1}$ return and
   $x_{k+2}(1) - x_{k+2}(0) = (a/4\pi)^kb/k!$ (eq. 12.31). *Sketch (4):* $[g_1, g_2] = -e_3$, induct;
   $\dot x_3 = x_2u_1$ multiplies sinusoids at $2k\pi$ and $2\pi$; product-to-sum $k$ times leaves a
   DC term only in $\dot x_{k+2}$; drive $x_1, x_2$ directly, then climb $k = 1..n-2$ with (12.31).
   *Collapsible:* Hitch with $p$ trailers in chained form [C ref. 393].
3. **Gradient steering converges when the end-state map has full rank** (C §12.5.4). *Statement:*
   with $e = f(x_{start}, u^p) - x_{goal}$ and $u^p \leftarrow u^p - \alpha(\partial f/\partial u^p)^\top e$,
   the iteration converges for small $\alpha$ if $\partial f/\partial u^p$ has rank $n$ everywhere —
   generically true for STLC systems with a rich parameterization. *Sketch (3):* $\|e\|^2$ decreases
   along $-\nabla$; rank $n$ makes every error direction correctable; at a singular $u^p$ append a
   generic loop. *Collapsible:* the Ch. 19 cross-link for §12.5.3's nonlinear program and its caveats.
4. **The car and the car with trailers are differentially flat** (C §12.5.5 [C refs. 152, 153, 309]).
   *Statement:* for the reduced car $y = (x_1, x_2)$, $x_3 = \operatorname{atan2}(\dot y_2, \dot y_1)$,
   $v = \pm\sqrt{\dot y_1^2 + \dot y_2^2}$, $\omega = (\dot y_1\ddot y_2 - \ddot y_1\dot y_2)/(\dot y_1^2
   + \dot y_2^2)$ (eq. 12.32–12.33); for the 4-state car $\phi = \arctan(L\kappa)$, $\kappa = \omega/v$;
   with trailers the flat outputs are the *last* trailer's axle midpoint. *Sketch (4):* heading is the
   path tangent; curvature is the signed cross product over speed cubed and $\tan\phi = L\kappa$; by
   eq. 12.36–12.37 the car's axle is the trailer axle plus $d_1$ times the trailer tangent, so
   differentiate once more per body; singular at $v = 0$. *Collapsible:* Example 12.5.1 in full.
5. **Shortest paths for the car: Dubins and Reeds–Shepp** (C §12.5.6 [C refs. 360, 402, 395, 394];
   Dubins 1957 (verify)). *Statement:* for the reduced car with $|v| \le v_{max}$, a shortest path
   lies in Choset's nine Reeds–Shepp families $\{C|C|C,\ CC|C,\ C|CC,\ CC_a|C_aC,\ C|C_aC_a|C,\
   C|C_{\pi/2}SC,\ CSC_{\pi/2}|C,\ C|C_{\pi/2}SC_{\pi/2}|C,\ CSC\}$ — forty-eight words (forty-six per
   [C ref. 402] (verify)) — and for the forward-only car in the six Dubins words $\{LSL, RSR, LSR, RSL,
   RLR, LRL\}$. *Sketch (6):* (i) Pontryagin: $H$ is linear in $\omega$, so $\omega$ is bang-bang or
   singular (straight); (ii) paths are full-lock arcs and straights; (iii) case analysis gives nine
   families; (iv) no cusps leaves $CSC$ and $CCC$; (v) **$LSL$ closed form, derived:** with $d = \|q_1 -
   q_0\|_{xy}/\rho$ and $\alpha, \beta$ the start and goal headings relative to the bearing of $q_1$:
   $p^2 = 2 + d^2 - 2\cos(\alpha - \beta) + 2d(\sin\alpha - \sin\beta)$, $t = -\alpha + \operatorname{atan2}
   (\cos\beta - \cos\alpha,\ d + \sin\alpha - \sin\beta)$, $q = \beta - \operatorname{atan2}(\cdot)$ — the
   two left circles and their external tangent; (vi) the other five by reflection and reversal.
   *Collapsible:* all six closed forms; the RS lookup by goal-plane region (Souères–Laumond synthesis).
6. **Local planners with the topological property make path transformation and PRM complete for
   cars** (C §12.5.6 [C refs. 267, 378]). *Statement:* if a free path exists for the free-flying car
   body with an open free neighborhood everywhere, recursive bisection with a Reeds–Shepp local
   planner terminates with a feasible path; parking into a space $\epsilon$ longer than the car needs
   $\Theta(1/\epsilon^2)$ cusps [C ref. 266]. *Sketch (4):* STLC + the topological property keep RS
   paths between close configurations in a prescribed ball; compactness gives a uniform $\epsilon$;
   bisect until consecutive samples are within $\epsilon$; PRM inherits probabilistic completeness
   because Ch. 11's proof never used symmetry of the local planner. *Collapsible:* curvature jumps.
7. **CAR GRID SEARCH is resolution complete and cost-optimal at its resolution** (C Alg. 22
   [C ref. 41]). *Statement:* best-first search over $\{L_\pm, R_\pm, S_\pm\}$ with a $d^3$ occupancy
   grid finds a path to $G(q_{goal})$ if one exists at small enough step, optimal for cost
   $a\cdot$steps $+ b\cdot$steering changes $+ c\cdot$cusps; any $p$-cusp path of the full car is
   approximable by a $p$-cusp path of full-lock arcs. *Sketch (4):* $d^3$ cells bound the occupied
   set (termination); bucketed best-first on integer costs is Dijkstra (Ch. 6); each step must leave
   its cell and the goal region must exceed a cell; full-lock approximation from Derivation 5.
   *Collapsible:* C Fig. 12.25's OPEN ordering (costs 3, 7, 7, 7, 8); the trailer extension.
8. **Forward propagation turns any simulator into a probabilistically complete planner** (C §7.5.1
   [C refs. 270–272, 195]). *Statement:* with $\mathrm{prop}$ Lipschitz in $x$ and controls sampled
   with full support, the single-tree RRT/EST that extends the nearest node toward a random state by
   the best sampled control reaches any goal region of nonzero measure with probability $\to 1$.
   *Sketch (5):* a feasible trajectory has a tube of nearby trajectories; discretize it into $K$ steps;
   each step is matched with positive probability from a node in the previous cross-section; the
   Voronoi bias (Ch. 12) keeps choosing frontier nodes; multiply and iterate. *Collapsible:* metric
   sensitivity [C refs. 272, 103]; time in the state [C ref. 195]; Kinodynamic RRT\* (verify): sampled
   controls replaced by an exact fixed-final-state, free-final-time LQR edge for linear(ized) dynamics.
9. **Decoupling vector fields and kinematic controllability** (C §12.4.2 [C refs. 283, 79]).
   *Statement:* $\nabla_{\dot q}\dot q = \sum Y_iu_i$ is maximally reducible iff $\mathrm{Sym}(\mathcal{Y})
   = \mathrm{span}(\mathcal{Y})$ (Thm. 12.4.5); $V$ is decoupling iff $V \in \mathrm{span}(\mathcal{Y})$ and
   $\nabla_VV \in \mathrm{span}(\mathcal{Y})$ (Thm. 12.4.7); STLKC ⇒ equilibrium controllable by
   decoupling-field segments with stops at switches. *Sketch (4):* substitute $\dot q = Vw$, $\ddot q =
   V\dot w + (\partial V/\partial q)Vw^2$; feasibility for all $w, \dot w$ forces $V$ and $\nabla_VV$ into
   the input span; solve $\langle X_c, \nabla_VV\rangle = 0$ on the $M$-orthogonal complement; plan in
   $\mathcal{Q}$, then time-scale with Ch. 18. *Collapsible:* Reach's 3R with $u_3 = 0$ — the free third
   link is the planar body with thrusters; translate along it or rotate about its center of percussion.

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| `dubins_shortest_path` / `reeds_shepp_shortest_path` | `(q0, q1, ρ) -> (Word, segs, len)` | $O(1)$: six / forty-eight closed forms (reflection, reversal, time-flip) |
| `steer_brockett` (§12.5.1) / `steer_sinusoids` (§12.5.2) | `(x3_goal) -> (u1(0), u2(0), k)` / `(x_start, x_goal, &Chained) -> Vec<Segment>` | $O(1)$ / $n - 2$ cycles |
| `steer_gradient` (§12.5.4) / `steer_flat` + `lift` (§12.5.5) | `(prop, x_start, x_goal, u_p0, α, ε) -> Result<u_p, Singular>` / `(x_start, x_goal, t_f, degree) -> FlatCurve` | one propagation + $r$ sensitivities per iteration / linear solve of size degree$+1$ |
| **C Alg. 22 CAR GRID SEARCH**; `path_transform` ([C ref. 267]); `prm_with_rs` / `rrt_with_rs` | `(q_start, G, Δt, d, (a, b, c), cs) -> Result<Path, Failure>`; `(free_flying_path, &ReedsShepp, cs) -> Path`; Ch. 11/12 planners with `ReedsShepp: Steer` | $\le 6d^3$ expansions with bucketed OPEN; $O(2^{\text{depth}})$ RS queries; as Ch. 11/12 |
| `kinodynamic_rrt` (§7.5.1) / `kinodynamic_rrt_star` (verify) | `(prop, x_start, G, n, metric, rng) -> Option<Trajectory>` | $O(n\log n)$ NN + $n \cdot |U_{sampled}|$ propagations; RRT\* adds an LQR two-point BVP per edge |
| `state_lattice` / `hybrid_a_star` (verify); `decoupling_grid_search` (§12.5.7) | `(prims, grid, q_start, q_goal, h[, &ReedsShepp]) -> Path`; `(fields, q_start, G, ik, cs) -> Path` then `time_scale` | A\* over $d^3 \times |\text{prims}|$, one continuous state per cell; Alg. 22 with 4 primitives, cost = #switches |

**Numeric micro-example** (the contract with the unit tests in §5). *Dubins:* $q_0 = (0, 0, 0)$,
$q_1 = (3, 3, \pi/2)$, $\rho = 1$. The start's left circle is centered at $(0, 1)$, the goal's at
$(3, 3) + \rho(-\sin\tfrac\pi2, \cos\tfrac\pi2) = (2, 3)$; the centers are $2\sqrt2$ apart along
direction $\pi/4$, so $LSL$ has $t = \pi/4$, $p = 2\sqrt2$, $q = \pi/4$ and length $\pi/2 + 2\sqrt2
= 4.39922$. By integration: after the first arc Hitch is at $(0.70711, 0.29289)$ heading $\pi/4$; the
straight adds $(2, 2)$; the last arc about $(2, 3)$ ends at $(3, 3)$ heading $\pi/2$. The others are
longer ($LSR = RSL = 10.56698$, $LRL = 10.99557$, $RSR = 16.65243$; $RLR$ infeasible), and
Reeds–Shepp returns the same $LSL$. *Flat outputs:* Choset's Example 12.5.1 curve $y_1 = t$, $y_2 =
3t^2 - 2t^3$ at $t = 0.25$: $y = (0.25, 0.15625)$, $\dot y = (1, 1.125)$, $\ddot y = (0, 3)$, so $\theta
= \operatorname{atan2}(1.125, 1) = 0.84415$ rad ($48.37°$), $v = \sqrt{2.265625} = 1.50520$, $\omega =
3/2.265625 = 1.32414$, $\kappa = 0.87971$, and for wheelbase $L = 1$ the steering angle is $\phi =
\arctan 0.87971 = 0.72149$ rad ($41.34°$). At $t = 0.5$ the curve inflects: $\omega = \phi = 0$ exactly.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"same destination, six fares."** Colors per TOC.md: Hitch, trailer and executed
trajectory **robot (orange)**; planned paths **path (purple)**, methods told apart by dash pattern;
trees and lattices **graph (blue)**; bay poses **goal (red)**; start **start (green)**; pillars
**obstacle (slate)**; the car's $SE(2)$ C-obstacle slice **C-obstacle (amber, hatched)** behind a
disclosure. Every widget autoplays a seeded default with an SVG fallback.

- **Widget w21.1: Trailer Parker** — wasm-sim (flagship). Hitch with trailer in the Lot; drag start
  and bay. **Manipulates:** headline selector of steering method — flat-output polynomial /
  chained-form sinusoids / Reeds–Shepp primitives (Alg. 22 when obstacles are on); disclosure:
  polynomial degree, sinusoid amplitude cap, Alg. 22 weights $(a, b, c)$, obstacles, trailer, seed.
  **Observes:** three purple paths over one query; the orange execution with $\theta_1$ on a gauge
  reddening toward $\pm\pi/2$; the scoreboard (length, cusps, max $|\theta_1|$, time, clearance).
  **Misconception killed:** "there is *the* path for a car."
- **Widget w21.2: Flatness Follower** — interactive animation. **Manipulates:** headline: draw or
  drag the flat-output curve (rear axle, or trailer axle when hitched) as a Bézier; disclosure:
  derivative vectors, $L$, trailer. **Observes:** the full state recovered from the curve alone —
  heading from the tangent, steering from curvature, car position from the trailer path's tangent;
  curvature over $1/\rho$ turns red. **Misconception killed:** "planning a car needs its equations."
- **Widget w21.3: Lattice vs Hybrid A\*** — wasm-sim, two panels, same Lot and query.
  **Manipulates:** headline slider for grid resolution $d$; disclosure: primitive set (Alg. 22's six /
  a 16-primitive lattice), heuristic (Euclidean / RS length / hybrid A\*'s $\max$ of RS length and
  obstacle-aware 2D A\*), analytic-shot frequency, seed. **Observes:** blue expansions, one continuous
  state per cell as a dot, RS shots flashing purple, expansion and cost counters. **Misconception
  killed:** "hybrid A\* is a new algorithm."
- **Widget w21.4: Reeds–Shepp Atlas** — interactive animation. **Manipulates:** headline: drag the
  goal pose relative to a fixed start; disclosure: $\rho$, Dubins toggle, faint all-words overlay.
  **Observes:** the goal plane tinted by optimal word family (the Souères–Laumond synthesis emerging);
  the word flips at region boundaries while the length stays continuous. **Misconception killed:**
  "reverse just means a Dubins word run backward."
- **Widget w21.5: Forward-Propagation Tree** — wasm-sim. Hitch's *dynamic* model (acceleration and
  steering-rate inputs). **Manipulates:** headline: controls sampled per extension (1 = classic
  LaValle–Kuffner, 8 = best-of-sampled); disclosure: $\Delta t$, metric weights, Kinodynamic RRT\*
  on/off, seed. **Observes:** a blue tree with curved, velocity-colored edges; first solution in purple;
  rewires and a descending cost curve with RRT\* on; the metric slider distorting which nodes get
  extended. **Misconception killed:** "RRT needs a steering function."
- **Widget w21.6: Decoupling Drive** — wasm-sim. Reach's 3R on the Workbench, third motor off.
  **Manipulates:** headline: drive the third link with four primitives (translate $\pm$, rotate $\pm$
  about the center of percussion); disclosure: run the switch-minimizing search, time-scale with
  Ch. 18, replay, show torques. **Observes:** the third link behaving like a car with two actions, the
  arm following by inverse kinematics, a phase-plane inset with one actuator always saturated.
  **Misconception killed:** "an unactuated joint means an unplannable arm."

Dashboard layout: w21.1 full width at the head of the C section; w21.4 and w21.2 half-width beside
Derivations 5 and 4; w21.3 and w21.5 full width sharing one sticky `PlanReport` scoreboard; w21.6
beside Derivation 9.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35, `rand` 0.9 (`Pcg64`), `petgraph` 0.8 (lattice graphs), `pathfinding`
4.15 (cross-check of the lattice A\*), `parry2d` 0.30 via Ch. 2's `collide` (arc and segment
shape-casts), Ch. 19's GN/LM for `steer_gradient` with `argmin` as comparison; imports: `sampling`
(Ch. 11 `Steer`, kd-tree; Ch. 12 `Rrt`; Ch. 13 `RrtStar`), `search` (Ch. 6), `nonholonomic` (Ch. 20),
`dynamics` (Ch. 17), `trajectory` (Ch. 18), `robots` (Ch. 2), `cspace` (Ch. 4).

**Module plan** — extending `crates/nonholonomic/` and adding `crates/kinodynamic/`:
`nonholonomic/src/steer/{dubins, reeds_shepp, brockett, sinusoid, gradient, flat}.rs`;
`src/car_grid.rs` (C Alg. 22, bucketed OPEN, trailer extension); `src/transform.rs`;
`src/lattice/{primitives, state_lattice, hybrid_a_star}.rs`; `src/reduction.rs` (`is_decoupling`,
`stlkc`, `decoupling_grid_search`); `kinodynamic/src/{propagate, rrt, rrt_star}.rs` (owns `trait
Propagate`; Ch. 14's preview re-exports `KinoRrt`). Examples `dubins_check`, `flat_car`,
`park_six_ways`, `reach_dead_joint_drive`; tests `micro_example.rs`, `words_integrate.rs`.

**Key types & signatures** (compiles in spirit):

```rust
use manifold::SE2;  use sampling::{Steer, SteerResult};   // Ch. 5; Ch. 11's local-planner trait

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Word { LSL, RSR, LSR, RSL, RLR, LRL }

/// Dubins shortest path for the reduced car (C eq. 12.35). Segment lengths are normalized by ρ
/// so the six closed forms are scale-free; every returned word is checked by integration in tests.
pub struct DubinsPath { pub word: Word, pub segs: [f64; 3], pub rho: f64 }
impl DubinsPath { pub fn len(&self) -> f64; pub fn sample(&self, q0: &SE2, ds: f64) -> Vec<SE2>; }
pub fn dubins_shortest_path(q0: &SE2, q1: &SE2, rho: f64) -> Option<DubinsPath>;

pub struct Dubins { pub rho: f64 }   pub struct ReedsShepp { pub rho: f64 }   // RS: 48 words, cusps
impl Steer<SE2> for Dubins {
    /// Toward `b` from `a`, truncated at `max_len`: a word's prefix is itself feasible, which is
    /// what makes Dubins a legal RRT `extend` and PRM local planner.
    fn steer(&self, a: &SE2, b: &SE2, max_len: f64) -> Option<SteerResult<SE2>>;
}

/// Flatness: a C² rear-axle curve y(t) determines (x, y, θ, φ, v, ω) by differentiation (Derivation 4).
pub struct FlatCurve { pub coeffs: [nalgebra::DVector<f64>; 2], pub t_f: f64 }
pub struct CarState { pub pose: SE2, pub phi: f64, pub v: f64, pub omega: f64 }
pub fn steer_flat(x_start: &CarState, x_goal: &CarState, t_f: f64, degree: usize) -> FlatCurve;
pub fn lift(curve: &FlatCurve, t: f64, wheelbase: f64) -> CarState;   // `lift_trailer` adds θ₁

/// C Alg. 22 CAR GRID SEARCH, generic over the car model so the trailer is one type away.
pub struct GridSearchCost { pub a: u32, pub b: u32, pub c: u32 }   // steps, steering changes, cusps
pub fn car_grid_search<C: CarModel>(car: &C, q_start: &C::Config, goal: &GoalRegion<C::Config>, dt: f64,
    cells_per_dim: usize, cost: GridSearchCost, cs: &dyn collide::Collision<C::Config>, max_tree: usize)
    -> Result<Vec<(C::Config, Action)>, Failure>;                      // Action ∈ {L±, R±, S±}

/// §7.5.1: planning with a simulator instead of a steer.
pub trait Propagate {
    type State; type Control;
    fn propagate(&self, x: &Self::State, u: &Self::Control, dt: f64) -> Self::State;  // RK4 inside
    fn sample_control(&self, rng: &mut rand_pcg::Pcg64) -> Self::Control;
}
pub struct KinoRrt<P: Propagate> { pub sys: P, pub controls_per_extend: usize, pub dt: f64 /* … */ }
impl<P: Propagate> KinoRrt<P> {
    pub fn plan(&mut self, x_start: P::State, goal: &dyn Fn(&P::State) -> bool, n: usize,
                metric: &dyn Fn(&P::State, &P::State) -> f64, rng: &mut rand_pcg::Pcg64) -> Option<Trajectory<P>>;
}
// KinoRrtStar<P: Propagate + Linearize> adds Webb & van den Berg's (verify) LQR edges to RRT*.

/// §12.4.2: V is decoupling iff V ∈ span(Y) and ∇_V V ∈ span(Y)  (Thm. 12.4.7).
pub fn is_decoupling<const Q: usize>(v: &dyn VectorField<Q>, inputs: &[&dyn VectorField<Q>],
                                     model: &dynamics::Model<Q>, q: &SVector<f64, Q>, tol: f64) -> bool;
```

**Worked end-to-end example:** `cargo run --example dubins_check -p nonholonomic` prints the
micro-example — `LSL  t=0.785398 p=2.828427 q=0.785398  len=4.399223`, the four feasible
alternatives, and `integrate(LSL) -> (3.000000, 3.000000, 1.570796)`; `cargo run --example flat_car`
prints the $t = 0.25$ row `theta=0.844154 v=1.505199 omega=1.324138 kappa=0.879709 phi(L=1)=0.721491`
and the $t = 0.5$ row with `omega=0.000000`. `#[test] fn reproduces_micro_example()` asserts all of
it to $10^{-6}$. `tests/words_integrate.rs`: 1000 seeded pose pairs, every Dubins and Reeds–Shepp
word integrates to its endpoint within $10^{-9}$, RS length $\le$ Dubins length $\ge$ Euclidean
distance. `examples/park_six_ways.rs` (seed 21) runs the catalog on one trailer-parking query among
three pillars and prints w21.1's scoreboard (snapshot-tested); `examples/reach_dead_joint_drive.rs`
reproduces Choset's Table 12.1 robot, one actuator saturated per segment, against its $0.890\,$s.

**Runnable artifact:** the WASM build of `nonholonomic` + `kinodynamic` is w21.1–w21.6; the widgets
call the same `DubinsPath::sample`, `car_grid_search`, `hybrid_a_star` and `KinoRrt::plan` the prose
prints. The Integration lab is `park_six_ways` live — Hitch with trailer, randomized pillars, every
planner, one scoreboard — plus a failure button: a bay so tight that only Alg. 22 with $c = 0$ finds
it, in $\Theta(1/\epsilon^2)$ cusps, as Ch. 20 promised.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w21.1 | Trailer Parker | wasm-sim | nonholonomic + robots + collide + sampling, Pcg64 | drag poses, method selector, cost weights, obstacles/trailer toggles, seed | one query, several optimality criteria; trailer swing as hidden cost |
| w21.2 | Flatness Follower | interactive animation | nonholonomic::steer::flat + robots | draw/drag flat-output curve, $L$, trailer | state by differentiation; curvature limits; singular points |
| w21.3 | Lattice vs Hybrid A\* | wasm-sim | nonholonomic::lattice + search + collide | resolution, primitive set, heuristic, RS-shot frequency, seed | modern planners are Alg. 22 plus heuristic, lattice, analytic shots |
| w21.4 | Reeds–Shepp Atlas | interactive animation | nonholonomic::steer::{dubins, reeds_shepp} | drag goal pose, $\rho$, Dubins toggle | word-structured shortest paths; the synthesis partition |
| w21.5 | Forward-Propagation Tree | wasm-sim | kinodynamic + dynamics + sampling, Pcg64 | controls per extend, $\Delta t$, metric weights, RRT\* toggle, seed | planning with a simulator; metric sensitivity; optimality needs steering |
| w21.6 | Decoupling Drive | wasm-sim | nonholonomic::reduction + dynamics + trajectory + robots | four primitives, run search, time-scale, replay | kinematic reduction; switch-minimizing plans; one actuator saturated |
| — | Three-Hitches hook | animation (autoplay) | nonholonomic + kinodynamic, seeded | none (replay) | same destination, different fares |
| f21.7 | CAR GRID SEARCH tree and OPEN list | static-svg | build-time SVG | hover nodes for cost | C Fig. 12.25's bucketed best-first ordering |

## 7. Exercises & Extensions

1. **(F)** Derive the $RSL$ closed form from $LSL$ by reflection and reversal, show where $p^2 < 0$
   makes it infeasible, and prove that for $\|q_1 - q_0\|_{xy} \ge 4\rho$ no $CCC$ word is optimal.
2. **(F)** Prove all chained-form systems are flat with $(x_1, x_n)$ as outputs (Choset Problem 25)
   and give $\varphi, \psi$ explicitly for $n = 4$, relating $x_n$ to Hitch's trailer flat output.
3. **(C, predict-then-verify)** In w21.4 with $\rho = 1$, predict the optimal Dubins word and length
   for the goal $(2, -1, 0)$ before dragging there; verify, then toggle Reeds–Shepp and explain
   whether a cusp can help at this range.
4. **(C)** In w21.1, place the bay so the sinusoid planner's trailer swing exceeds $|\theta_1| = 1.2$
   rad while the flat planner stays under $0.6$; then make the flat planner fail (curvature over
   $1/\rho$) by shortening $t_f$, and say where control constraints enter each method.
5. **(P)** Implement the trailer extension of `car_grid_search` with a lookup table of
   $\Delta\theta_1$ per primitive (C §12.5.6), check it against integration on three seeded queries,
   and add the jackknife guard $|\theta_1| < \pi/2 - \delta$ as pruning, measuring expansions saved.
6. **(P, stretch)** Implement `KinoRrtStar` for Hitch's dynamic model linearized about rest (Webb &
   van den Berg (verify): fixed-final-state, free-final-time LQR edges); compare its cost curve with
   `KinoRrt` (best-of-8) over 20 seeds; then swap the LQR edge for `steer_flat` and report which
   assumption each connection violates.

## 8. Modernization Notes

- **Kept, in full:** C §12.5 is the only textbook survey putting optimal control, sinusoids,
  gradient steering, flatness, Reeds–Shepp, grid search, path transformation and kinematic
  reductions in one chapter with one running example; every method keeps its derivation, numeric
  contract and widget. Alg. 22 is implemented as written, bucketed OPEN included. §7.5.1's forward
  propagation, two paragraphs in Choset, grows into a derivation, a crate and a widget because it is
  now how kinodynamic planning is mostly done (OMPL's control-based planners).
- **Added beyond 2005 (tagged for verification until checked):** state lattices (Pivtoraiko, Knepper
  & Kelly) and hybrid A\* (Dolgov, Thrun, Montemerlo & Diebel) presented *explicitly* as descendants
  of Alg. 22 and of Choset's PRM-with-Reeds–Shepp local planner, with the three deltas named
  (lattice-consistent primitives; one continuous state per cell plus analytic RS shots; the dual
  heuristic); Kinodynamic RRT\* (Webb & van den Berg) as RRT\* with an optimal-control steer; Dubins
  1957. The sister book's Dubins section is a preview; here $LSL$ is derived and every word is tested.
- **Dropped or condensed:** §12.5.3 is a cross-link to Ch. 19 plus Choset's two caveats; §12.5.8's
  fictitious inputs and motion libraries are one pointer box each (the maneuver automaton as the
  lattice's ancestor); eq. 12.38 is stated for $p$ trailers but implemented for one, Hitch's
  configuration; Choset's Fig. 12.31 is described, not reproduced.
- **Honesty items the prose must keep:** Alg. 22 is resolution complete, not exact; Reeds–Shepp and
  Alg. 22 paths have curvature jumps a real steering actuator cannot follow; flatness handles neither
  control bounds nor obstacles without numerical optimization; forward propagation is metric-sensitive
  and offers no optimality without an exact steer; the kinematic-reduction planner stops at every
  switch, forfeiting global time-optimality; the $\Theta(1/\epsilon^2)$ cusp count is a theorem.
