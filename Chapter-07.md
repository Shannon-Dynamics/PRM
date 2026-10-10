# Chapter 7 — Potential Functions

> Part II — Classical Planners: Search, Potentials, Roadmaps, Cells · Estimated length: 10 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Chapter 6 planned on a graph someone had already built. This chapter asks what a robot can do with
nothing but a goal and a sense of distance: treat the goal as a valley, the obstacles as hills, and
roll downhill. The "aha" comes in two halves. First, the attractive/repulsive potential is
beautifully local — $\nabla U$ needs only the goal and the nearest obstacle, so Rusty computes it
from a range scan (Ch. 3) and Reach from a few link points lifted through $J^\top$ (Ch. 4) — and it
is *incomplete*, because a sum of a valley and hills can have a second valley that is not the goal.
Second, the cure is not to tweak the hills but to change what a "potential" is: **a
distance-to-goal field has exactly one minimum**, computed on a grid by the BFS the reader already
owns — wave-front is Chapter 6's BFS rooted at the goal, brushfire the same BFS rooted at the
obstacles. Navigation functions (Rimon–Koditschek) show the single minimum can even be had smoothly
on a restricted family of worlds, so the local-minimum disease is a fact about *additive*
potentials, not about potentials. Reader path: Hook → C → F → P → Integration lab → Exercises.

**Story line**
1. **Hook** — a bead (Rusty as a point) slides down $-\nabla U$, swerves around a disc, arrives.
   Replace the disc by a horseshoe and it stops in the mouth (autoplay, C Fig. 4.10). Nothing is
   broken; the mathematics is doing exactly what it says.
2. **Play (C)** — w7.1 Potential Well: drag goal and horseshoe, watch the stuck badge; flip to
   wave-front mode and the bead escapes.
3. **Formalize (F)** — potentials, critical points, the Hessian; $U_{att}$, $U_{rep}$; C Alg. 4;
   sensor and grid distance; local minima; wave-front; navigation functions; lifting via $J^\top$.
4. **Algorithm** — gradient descent, brushfire, wave-front, navigation functions, $J^\top$ lifting.
5. **Implementation (P)** — the `potential` crate: `trait Potential`, attractive and repulsive
   terms over Ch. 4 distance queries, the owned grid distance transform, navigation functions,
   `LiftedPotential` for Reach.
6. **Integration lab** — Rusty in the Apartment under a sensor-based potential, stuck in a doorway
   alcove, freed by the wave-front field; Reach pulled by control-point forces on the Workbench.
7. **Close** — the first *field* planner; pointers to Ch. 8 (fronts collide on the GVD), Ch. 19
   (CHOMP descends a distance field), and the sister book's ESDF chapter.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (Rusty, Reach, Apartment, Workbench, `collide` distance queries, Reach's
  FK), Ch. 3 ($\rho(x,s)$, $D(x)$, $\nabla D$ — imported for §4.3.1), Ch. 4 ($d_i(q)$ via `parry2d`;
  `jacobian()` — imported for §4.7.3), Ch. 5 (gradients on manifolds, $T^2$), Ch. 6 (BFS —
  brushfire and wave-front *are* BFS; `search` is imported).
- **Feeds into:** Ch. 8 (brushfire ridges are the grid GVD; Morse vocabulary formalized there),
  Ch. 9 (OPP traces maxima of $D$ on slices), Ch. 10 (brushfire and wave-front decompositions,
  §6.2.3), Ch. 19 (CHOMP's obstacle term is a distance field), Ch. 23 (reactive layer).
- **Baseline sources:** C Ch. 4 — intro (potential, gradient, critical points, Hessian, Fig. 4.2;
  b. 77–80), §4.1 Eqs. (4.1)–(4.7) (b. 80–84), §4.2 Alg. 4 (b. 84–85), §4.3.1 mobile-robot
  implementation, §4.3.2 brushfire (b. 85–89), §4.4 local minima, RPP [C ref. 37] (b. 89–90), §4.5
  wave-front (b. 90–92), §4.6 Def. 4.6.1, §4.6.1 sphere-space Eqs. (4.8)–(4.13), §4.6.2 star-space
  Eqs. (4.14)–(4.19) [C refs. 239, 364] (b. 93–99), §4.7.1 virtual work $J^\top f = u$ and Example
  4.7.1, §4.7.2 control points, $r_{float}$, Eqs. (4.24)–(4.26), §4.7.3 articulated bodies
  (b. 99–104). Sister book: Ch. 19 (`…/chapters/ch19-map-representations`, exact EDT — the `edt`
  port), Ch. 20 (`…/chapters/ch20-motion-planning`, its Potential Well preview), Ch. 23
  (`…/chapters/ch23-mppi`, reactive control absorbing the potential's role). Modernization set:
  none required; CHOMP (Ratliff et al. 2009, verify) is a forward pointer only.

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; $U$, $U_{att}$, $U_{rep}$, $\nabla U$, $D(x)$, $d_i(q)$
are book-wide):

| Symbol | Meaning |
|---|---|
| $\zeta$, $\eta$; $Q^\star$, $Q^\star_i$ | attractive and repulsive gains; repulsive distance of influence (global; per obstacle) |
| $d^\star_{goal}$ | radius where the attractive potential switches from quadratic to conic |
| $c$; $\alpha(i)$ | closest point on $\mathcal{QO}_i$ to $q$, so $\nabla d_i(q) = (q - c)/d_i(q)$; descent step size |
| $\nabla^2 U$ | Hessian; nondegenerate critical point $\Leftrightarrow$ nonsingular Hessian |
| $\beta_i(q)$, $\beta(q)$ | obstacle functions ($<0$ inside, $0$ on $\partial\mathcal{QO}_i$); product $\prod_i \beta_i$ |
| $\gamma_\kappa$, $\sigma_\lambda$, $\xi_\kappa$ | goal term $d^{2\kappa}$, analytic switch $x/(\lambda+x)$, sharpening $x^{1/\kappa}$ |
| $\varphi(q)$; $h_\lambda$, $T_i$, $\nu_i$, $s_i$ | navigation function (C Def. 4.6.1), $\varphi = \xi_\kappa \circ \sigma_1 \circ \gamma_\kappa/\beta$; star-to-sphere diffeomorphism, translated scaling maps, switches |
| $r_j(q)$, $J_j(q)$; $r_{float}$ | control point $j$ and its Jacobian, $u = J_j^\top f$; floating control point (robot boundary point nearest any obstacle) |

**Definitions** (display boxes):
- *Potential function; gradient descent path* (C Ch. 4 intro): differentiable $U : \mathbb{R}^m \to
  \mathbb{R}$, $\nabla U$ a velocity (first-order robots) or force (§4.7); $\dot c = -\nabla U(c)$,
  stopping where $\nabla U(q^\star) = 0$.
- *Critical point and its type*: a zero of $\nabla U$; minimum / maximum / saddle as the Hessian is
  positive definite / negative definite / indefinite; *nondegenerate* if the Hessian is nonsingular
  (hence isolated). The full Morse vocabulary is Ch. 8's box; this chapter uses only "nondegenerate
  critical point," as C §4.6 does when it says a navigation function "is Morse."
- *Distance to an obstacle*: $d_i(q) = \min_{c \in \mathcal{QO}_i} d(q,c)$ (C Eq. 4.6);
  $D(q) = \min_i d_i(q)$.
- *Navigation function* (C Def. 4.6.1): $\varphi : \mathcal{Q}_{free} \to [0,1]$, at least $C^2$,
  unique minimum at $q_{goal}$ in its component, maximal on $\partial \mathcal{Q}_{free}$, Morse.
- *Sphere world, star-shaped set* (C §4.6.1–4.6.2): a disc bounded by a sphere containing disjoint
  spherical obstacles; a set with a point that sees every other point of the set along a segment.
- *Control point* (C §4.7.2): a point $r_j$ on the robot whose workspace potential is lifted to a
  configuration-space force by $J_j^\top$; enough of them to pin the body down (two in the plane).

**Derivations / theorems:**

1. **The blended attractive potential has a continuous gradient** (C §4.1, Eqs. 4.2–4.3).
   *Statement:* with $U_{att} = \tfrac12 \zeta d^2$ for $d \le d^\star_{goal}$ and
   $d^\star_{goal}\zeta d - \tfrac12 \zeta (d^\star_{goal})^2$ beyond, $\nabla U_{att}$ is continuous
   and bounded by $\zeta d^\star_{goal}$. *Sketch (3 steps):* (i) inside, $\zeta(q - q_{goal})$;
   (ii) outside, $d^\star_{goal}\zeta (q - q_{goal})/d$; (iii) at $d = d^\star_{goal}$ both equal
   $\zeta (q - q_{goal})$. *Collapsible:* the conic potential chatters at the goal (magnitude
   $\zeta$, undefined direction); the quadratic one grows without bound.
2. **Gradient of distance to a convex obstacle** (C Eq. 4.7). *Statement:* if $c$ is the unique
   closest point of convex $\mathcal{QO}_i$ to $q \notin \mathcal{QO}_i$, then $\nabla d_i(q) = (q -
   c)/\|q - c\|$, a unit vector. *Sketch (4 steps):* (i) $\min_c \|q - c\|$ over a convex set has a
   unique minimizer; (ii) envelope theorem: differentiate at the fixed minimizer;
   (iii) $\partial_q \|q - c\| = (q - c)/\|q - c\|$; (iv) convexity enters through uniqueness —
   nonconvex obstacles make $d_i$ nonsmooth where two closest points tie (C §4.1's oscillation).
   *Collapsible:* $U_{rep} = \sum_i U_{rep_i}$ (C §4.1); $d_i$ as a *local* minimum of
   $\rho(q,\theta)$ over $\theta$ (C §4.3.1, imported from Ch. 3).
3. **Additive potentials admit non-goal minima** (C §4.4, Figs. 4.10–4.11). *Statement:* for two
   disc obstacles symmetric about the line from $q$ to $q_{goal}$, a point on that line has
   $\nabla U = 0$ with positive definite Hessian. *Sketch (4 steps):* (i) by symmetry the repulsive
   gradient on the axis is axial; (ii) it grows without bound toward the gap while the attractive
   gradient stays bounded; (iii) the intermediate value theorem gives a zero; (iv) both repulsive
   terms push back toward the axis, so the point is transversally stable. *Collapsible:* the
   horseshoe case; RPP's random walks [C ref. 37]; the method is not complete.
4. **Brushfire computes grid distance and wave-front computes grid cost-to-goal; both are BFS**
   (C §4.3.2, §4.5; Ch. 6 Derivation 1). *Statement:* labeling obstacle pixels 1, their free
   neighbors 2, theirs 3, … assigns each pixel $1 + $ its link length to the nearest obstacle
   (Manhattan for 4-point, chessboard for 8-point); seeding the goal with 2 gives $2 + $ link length
   to the goal, whose descent is a shortest grid path. *Sketch (3 steps):* (i) multi-source BFS with
   all obstacle pixels at depth 0; (ii) Ch. 6 Derivation 1: depth = shortest link length; (iii) on a
   bounded grid every free pixel gets a label and every non-goal pixel has a neighbor labeled one
   less, so descent ends at the goal (C §4.5). *Collapsible:* "point to a lowest neighbor"; fronts
   colliding on equidistant pixels (Ch. 8's GVD); voxels and the exponential cost in dimension (C §4.5).
5. **The sphere-world navigation function** (C §4.6.1, Eqs. 4.8–4.13 [C refs. 239, 364]).
   *Statement:* $\varphi(q) = d^2(q,q_{goal}) / [d^{2\kappa}(q,q_{goal}) + \beta(q)]^{1/\kappa}$ is a
   navigation function on a sphere world for sufficiently large $\kappa$. *Sketch (5 steps):*
   (i) $\gamma_\kappa/\beta$ is zero only at the goal and $+\infty$ on every obstacle boundary;
   (ii) for large $\kappa$ the goal term dominates $\partial\beta$ away from obstacles, so $\nabla$
   points goalward; (iii) near an obstacle only it matters, so a spurious critical point can lie
   only on the radial line to the goal, where the Hessian is not positive definite — spurious minima
   become saddles; (iv) $\sigma_1$ squashes the range to $[0,1]$ without moving critical points;
   (v) $\xi_\kappa$ sharpens them to nondegeneracy. *Collapsible:* the $\kappa$ sweep of C Figs.
   4.15–4.16 (three minima at $\kappa=3$, one at $10$); the plateaus (C §4.6.1 last paragraph).
6. **Navigation functions pull back under diffeomorphisms; the star-to-sphere map** (C §4.6.2,
   Eqs. 4.14–4.19). *Statement:* if $\varphi$ is a navigation function on $M$ and $h : F \to M$ is a
   diffeomorphism, $\varphi \circ h$ is one on $F$; for star worlds $h_\lambda = s_{q_{goal}}
   \mathrm{id} + \sum_i s_i T_i$ is such a map for suitable $\lambda$. *Sketch (4 steps):*
   (i) critical points correspond one-to-one under $h$, nondegeneracy preserved; (ii) $T_i(q) =
   \nu_i(q)(q - q_i) + p_i$ sends $\partial\mathcal{QO}_i$ (where $\beta_i = 0$) onto a sphere of
   radius $r_i$; (iii) the switches $s_i$ are 1 on $\partial\mathcal{QO}_i$ and 0 at the goal and on
   other boundaries, so $h_\lambda = T_i$ exactly where it must; (iv) smoothness, bijectivity, and a
   smooth inverse for suitable $\lambda$ are quoted from [C ref. 239]. *Collapsible:* virtual work
   $f^\top J \dot q = u^\top \dot q \Rightarrow u = J^\top f$ (C §4.7.1); Example 4.7.1's torque
   $u_\theta = -f_x(a_x \sin\theta + a_y\cos\theta) + f_y(a_x\cos\theta - a_y\sin\theta) =
   (r \times f)_z$; lifted forces summed in configuration space, never the workspace (C Eq. 4.26).

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| C Alg. 4 Gradient Descent | `(U, q_start, alpha, eps) -> Vec<q>` | per step $O(\#\text{obstacle terms})$ |
| Brushfire (C §4.3.2) | `(grid, conn) -> DistGrid` | $O(\#cells)$, multi-source BFS |
| Wave-front planner (C §4.5) | `(grid, goal, conn) -> (CostGrid, descent path)` | $O(\#cells)$ |
| Sensor-based distance (C §4.3.1) | `(scan) -> Vec<(d_i, grad_i)>` | $O(\#beams)$ local-minimum scan |
| Navigation function: sphere world (C Eq. 4.13), star world via $h_\lambda$ (C Eq. 4.19) | `(world, kappa[, lambda]).value/gradient(q)` | $O(n)$ per evaluation (+ chain rule) |
| Control-point lifting (C Eq. 4.26) | `(robot, points, U_ws).gradient(q) = Σ J_jᵀ f_j` | $O(\#points \cdot \dim\mathcal{Q})$ |

**Numeric micro-example** (the chapter's contract with the unit test). *Part A — brushfire.* A
$7 \times 3$ grid, cells $(col,row)$, obstacles at $(1,2)$, $(7,2)$ (value 1), 4-point connectivity:

```
row 3:  2 3 4 5 4 3 2
row 2:  1 2 3 4 3 2 1
row 1:  2 3 4 5 4 3 2
```

Each label is $1 +$ Manhattan distance to the nearer obstacle. Column 4 is equidistant from both:
those three pixels get two back pointers — the fronts collide there; Ch. 8 calls that column the
grid GVD. The grid gradient at $(5,3)$ points to $(6,3)$ or $(5,2)$ (both 3; first in scan order).
*Part B — one potential evaluation.* Point robot at $q = (0,0)$, $q_{goal} = (4,3)$, $\zeta = 1$,
quadratic attractive term ($d^\star_{goal} > 5$): $U_{att} = \tfrac12 \cdot 25 = 12.5$,
$\nabla U_{att} = (q - q_{goal}) = (-4, -3)$. One disc obstacle, center $(0,-2)$, radius 1:
closest point $c = (0,-1)$, $d_1 = 1$, $\nabla d_1 = (0, 1)$; with $\eta = 1$, $Q^\star = 2$:
$U_{rep} = \tfrac12 (1/1 - 1/2)^2 = 0.125$, $\nabla U_{rep} = (1/2 - 1/1)\cdot(1/1^2)\cdot(0,1) =
(0, -0.5)$. Total $U = 12.625$, $\nabla U = (-4, -3.5)$. One descent step with $\alpha = 0.1$:
$q(1) = q - \alpha \nabla U = (0.4, 0.35)$ — away from the obstacle, toward the goal.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, carried end to end: **"roll downhill — and ask who built the hill."** Colors per
the book code: start green, goal red, obstacles slate, Rusty/Reach and the executed descent orange,
the wave-front field in the graph color's tints (it is an explored set), the final path purple.

- **Widget w7.1: Potential Well** — type: wasm-sim. A heightfield of $U$ over a Workbench-sized
  square with a draggable goal and horseshoe (two discs behind a disclosure); a bead (Rusty as a
  point) follows $-\nabla U$ at fixed $\alpha$ from a seeded start. **Manipulates:** headline — the
  repulsive influence radius $Q^\star$; disclosure: $\zeta$, $\eta$, $\alpha$, 4/8 connectivity for
  wave-front mode, seed re-roll. **Observes:** the bead trail; a "stuck: $\|\nabla U\| < \epsilon$,
  not at goal" badge with Hessian eigenvalue signs; toggling **Wave-front mode** swaps the
  heightfield for the BFS cost-to-goal (Derivation 4) and the bead escapes along a grid-shortest
  path. **Misconception killed:** "local minima are caused by concave obstacles" — two convex
  discs suffice (C Fig. 4.11); the disease is in the sum.
- **Widget w7.2: Navigation Function Sculptor** — type: interactive animation. The five-obstacle
  sphere world of C Fig. 4.14 with contours of $\varphi$. **Manipulates:** headline — $\kappa$
  slider $2 \to 12$; disclosure: "star mode" morphing each disc into a star-shaped blob with
  $h_\lambda$ drawn as a displacement field, $\lambda$ slider, goal drag. **Observes:** spurious
  minima migrating toward obstacles, becoming saddles, vanishing; a live count of critical points by
  type; in star mode the contours of $\varphi \circ h_\lambda$ and the one-to-one correspondence of
  critical points. **Misconception killed:** "potentials always have local minima" — on sphere and
  star worlds there is a smooth field with exactly one.
- **Widget w7.3: Arm in a Field** — type: wasm-sim. Reach on the Workbench with two attractive
  control points on the end effector and repulsive ones on each link plus $r_{float}$.
  **Manipulates:** headline — toggle $r_{float}$; disclosure: per-point gains $\zeta_j$, $\eta_j$,
  link control-point count, goal pose. **Observes:** workspace force arrows; the lifted torques
  $u = \sum_j J_j^\top f_j$ as a joint-space bar chart; the same gradient on the unwrapped torus
  beside the Ch. 4 C-obstacle; a mid-link collision when $r_{float}$ is off (C Fig. 4.20).
  **Misconception killed:** "repelling the vertices keeps the body safe."
- **Widget w7.4: Brushfire Fronts** — type: interactive animation. The §3 micro-grid, then an
  Apartment room: fronts grow from the obstacles one label per tick. **Manipulates:** headline —
  4- vs. 8-point connectivity; disclosure: paint obstacles, tick scrub. **Observes:** labels
  appearing; two-back-pointer pixels lit as fronts meet (Ch. 8 preview); diamond-vs-square growth.
  **Misconception killed:** "brushfire computes Euclidean distance" — it computes a grid metric;
  the exact EDT (sister Ch. 19) is a different algorithm.

Dashboard layout: w7.1 full-width atop the C section; w7.2 and w7.4 half-width beside Derivations
5 and 4; w7.3 full-width in §4.7, torus beside torque bars. All autoplay seeded defaults, static SVG fallback.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`SVector` configurations, Jacobians), `parry2d` 0.30 (`parry2d-f64`;
distance and closest-point queries through the Ch. 2 `collide` trait — not hand-rolled), `rand`
0.9 (`SmallRng` seeded starts), `search` (Ch. 6 BFS), `cspace` (Ch. 4 $d_i(q)$, `jacobian()`),
`bugs` (Ch. 3 range-scan geometry for the sensor-based distance).

**Module plan:**
- `crates/potential/` — **introduced here; `brushfire` is the owned grid distance transform
  imported by Chs. 8 and 10.** `src/lib.rs` (`trait Potential`), `src/attractive.rs`,
  `src/repulsive.rs`, `src/descent.rs` (C Alg. 4), `src/sensor_distance.rs` (§4.3.1),
  `src/brushfire.rs` (`brushfire`, `DistGrid`, `ridge_cells`), `src/wavefront.rs`,
  `src/navigation.rs` (`SphereWorld`, `StarWorld`, `h_lambda`), `src/lifted.rs`
  (`LiftedPotential`), `examples/{descent_step, brushfire_7x3, kappa_sweep, reach_field}.rs`.
- `web/lib/potential/` — TypeScript port for w7.1–w7.4; `__checks__.ts` pins Part A and Part B.

**Key types & signatures:**

```rust
use nalgebra::{SVector, Point2};

/// A differentiable scalar field over an N-dimensional configuration space; gradients are
/// read as velocities (first-order robots) — C Ch. 4's convention.
pub trait Potential<const N: usize> {
    fn value(&self, q: &SVector<f64, N>) -> f64;
    fn gradient(&self, q: &SVector<f64, N>) -> SVector<f64, N>;
}
/// C Eqs. (4.2)–(4.3): quadratic inside `d_goal_star`, conic outside (seam checked, Derivation 1).
pub struct Attractive<const N: usize> { pub goal: SVector<f64, N>, pub zeta: f64, pub d_goal_star: f64 }
/// C Eqs. (4.4)–(4.5) summed per obstacle (C §4.1) so the nearest-point switch never oscillates;
/// `dist` is the Ch. 4 query (distance to C-obstacle i and its closest point c).
pub struct Repulsive<'w, const N: usize> { pub dist: &'w dyn cspace::DistanceQuery<N>, pub eta: f64, pub q_star: f64 }
/// `Additive<N>(Vec<Box<dyn Potential<N>>>)` sums terms — the whole method and its local minima.
/// C Alg. 4 with the sign as the text intends: q(i+1) = q(i) - alpha * grad U(q(i)).
/// `DescentOutcome` is Goal | LocalMinimum { hessian_eigs } | StepLimit.
pub fn gradient_descent<const N: usize>(
    u: &dyn Potential<N>, q_start: SVector<f64, N>, alpha: impl Fn(usize) -> f64,
    eps: f64, max_steps: usize,
) -> (Vec<SVector<f64, N>>, DescentOutcome);

// src/brushfire.rs — the owned distance transform (C §4.3.2)
pub enum Conn { Four, Eight }
pub struct DistGrid { pub label: Vec<u32>, pub back: Vec<Option<(usize, usize)>>, pub w: usize, pub h: usize }
/// Obstacles 1, free 0 in; labels 1 + link length out; cells two fronts reach keep both back pointers.
pub fn brushfire(occ: &cspace::Raster, conn: Conn) -> DistGrid;
impl DistGrid {
    /// Pixels whose two back pointers name different obstacles — Ch. 8's grid GVD.
    pub fn ridge_cells(&self) -> Vec<(usize, usize)>;
    pub fn grid_gradient(&self, cell: (usize, usize)) -> (i8, i8); // "point to a lowest neighbor" (C §4.3.2)
}
pub fn wavefront(occ: &cspace::Raster, goal: (usize, usize), conn: Conn) -> DistGrid; // C §4.5: goal seeded with 2

// src/navigation.rs — C Eq. (4.13); `Potential<2>` impl via the quotient rule
pub struct SphereWorld { pub center: Point2<f64>, pub r0: f64, pub obstacles: Vec<(Point2<f64>, f64)> }
pub struct NavigationFunction<'w> { pub world: &'w SphereWorld, pub goal: Point2<f64>, pub kappa: u32 }

// src/lifted.rs — C §4.7.2–4.7.3 for Reach (Ch. 2 FK, Ch. 4 Jacobian); `Role` is Attractive{zeta} |
// Repulsive{eta, q_star}; `gradient` returns u(q) = Σ_j J_j(q)ᵀ f_j, summed in C-space (C Eq. 4.26).
pub struct ControlPoint { pub link: usize, pub local: Point2<f64>, pub role: Role }
pub struct LiftedPotential<'r> { pub arm: &'r robots::Reach, pub points: Vec<ControlPoint>, pub floating: bool }
```

**Worked end-to-end example:** `cargo run --example brushfire_7x3 -p potential` prints the three
label rows of Part A and `ridge: (4,1) (4,2) (4,3)`; `#[test] fn brushfire_7x3_labels()` asserts
all 21 labels and the ridge set; `#[test] fn brushfire_equals_bfs_depth_plus_one()` checks on 100
seeded grids that `brushfire` equals `search::bfs` multi-source depth $+1$ and that the wave-front
descent path has the Ch. 6 Dijkstra cost under unit weights. `cargo run --example descent_step`
prints `U_att = 12.5  grad = (-4, -3)`, `U_rep = 0.125  grad = (0, -0.5)`, `U = 12.625  grad =
(-4, -3.5)`, `q1 = (0.4, 0.35)`; `#[test] fn descent_step_matches_text()` pins them to $10^{-12}$.
`kappa_sweep` counts critical points of the five-disc navigation function for $\kappa =
3,4,6,7,8,10$ by Newton polishing from a seeded grid of starts, printing `kappa=3: minima=3 …
kappa=10: minima=1` (pinned: exactly one minimum at $\kappa = 10$). `reach_field` lifts a unit
force at Reach's tip through $J^\top$ and asserts $u_\theta = (r \times f)_z$ (Example 4.7.1).

**Runnable artifact:** w7.1 and w7.4 are `web/lib/potential/` ports of `Additive`,
`gradient_descent`, `brushfire`, and `wavefront`; w7.2 ports `NavigationFunction`; w7.3 ports
`LiftedPotential` over the ported Reach FK/Jacobian. The Integration lab runs Rusty in the Apartment
with `sensor_distance` feeding `Repulsive` from Ch. 3 scans, records the alcove where it stalls,
then escapes with `wavefront` on the Ch. 4 raster; Reach descends `LiftedPotential` on the
Workbench with and without $r_{float}$, logging the first collision.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w7.1 | Potential Well | wasm-sim | `potential` → `web/lib/potential/`, `search` BFS, `SimCanvas` | drag goal/obstacles, $Q^\star$ slider, wave-front toggle, seed re-roll | local minima are a property of sums; a cost-to-go field has one minimum |
| w7.2 | Navigation Function Sculptor | interactive animation | `potential::navigation` → TS port | $\kappa$ slider, star mode, $\lambda$, goal drag | spurious minima become saddles; diffeomorphisms carry navigation functions |
| w7.3 | Arm in a Field | wasm-sim | `potential::lifted`, `robots::Reach`, `cspace::jacobian` → TS port | $r_{float}$ toggle, gains, goal pose | $u = J^\top f$; vertices alone do not protect a body |
| w7.4 | Brushfire Fronts | interactive animation | `potential::brushfire` → TS port | connectivity toggle, paint, tick scrub | brushfire is BFS; fronts meet on the GVD; grid metric $\ne$ Euclidean |
| — | Bead-in-horseshoe hook | animation (autoplay) | `potential`, `sim` | none (replay) | the method works, then stops; nothing is broken |

## 7. Exercises & Extensions

1. **(F)** Prove that $d_i$ is convex when $\mathcal{QO}_i$ is convex (C Ch. 5 Problem 19 brought
   forward) and conclude that $U_{rep_i}$ is nonsmooth only where $D$ switches obstacle.
2. **(F)** Show the wave-front path is shortest in the $L_1$ metric for 4-point connectivity
   (C Ch. 4 Problem 2), and give a $3 \times 3$ example where the 8-point path is not Euclidean-shortest.
3. **(C, predict-then-verify)** In w7.2, read the critical-point count at $\kappa = 6$; predict the
   smallest $\kappa$ with one minimum before moving the slider; verify, then drag one obstacle
   toward the goal and explain which way the threshold moves.
4. **(C)** In w7.1 build a world of only convex discs in which the bead gets stuck. Enable
   wave-front mode and record how close the escape passes to obstacles — C §4.5's "dangerously close."
5. **(P)** Implement the 8-point `brushfire` with diagonal cost $1.4$ (Ch. 6 metric) as a Dijkstra
   rather than a BFS; property-test it within one cell of the exact EDT ported from `mapping/edt`.
6. **(P, stretch)** Implement C Ch. 4 Problem 5: the two-link arm's wave-front planner on Reach's
   $T^2$ raster from Ch. 4 (wrap-around across the seam), animating the arm on the Workbench.

## 8. Modernization Notes

- **Kept from Choset, deliberately:** the additive construction with its honest failure (C §4.4),
  because every later field method (CHOMP, reactive controllers) inherits its local-minimum
  structure; $d_i$ as local minima of $\rho$ (§4.3.1), Rusty's first *online* planner; brushfire and
  wave-front as first distance transforms; navigation functions in full (Def. 4.6.1, sphere and star
  worlds), which prove the single-minimum property attainable and introduce the "critical points
  under diffeomorphisms" idea Chs. 8–10 rely on; §4.7's $J^\top$ lifting, reused in Ch. 17.
- **Added beyond 2005:** brushfire and wave-front identified with Ch. 6's BFS as a theorem; grid
  distance distinguished from the exact EDT (sister Ch. 19's `edt`, ported, Exercise 5); Hessian
  eigenvalue reporting at termination (the stuck badge); the link from local minima to
  trajectory-optimization non-convexity (CHOMP, Ratliff et al. 2009, verify; Ch. 19) and to
  sampling-based planners as the field's eventual answer (Part III).
- **Dropped/condensed:** RPP [C ref. 37] to one paragraph (random walks reappear as sampling in
  Ch. 11); voxel generalization to one remark; C Fig. 4.9's label propagation to the $7 \times 3$
  micro-grid and w7.4.
- **Honesty items the prose must keep:** (i) C Alg. 4 line 4 prints $q(i+1) = q(i) + \alpha(i)
  \nabla U$ while the text says "opposite the gradient" — a sign typo; our code uses the minus;
  (ii) the navigation-function guarantee is existential in $\kappa$ ("sufficiently large") and comes
  with the plateau problem C §4.6.1 admits; (iii) control points approximate the body — $r_{float}$
  reduces but does not eliminate the risk of C Fig. 4.20; (iv) every potential planner here is
  incomplete except wave-front, which is resolution complete and exponential in dimension.
