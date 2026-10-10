# Chapter 4 — Configuration Space I: Obstacles in the Space of Configurations

> Part I — Foundations: Robots, Worlds, and Configuration Space · Estimated length: 10 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Chapter 1 promised that the piano is a point; this chapter pays. It answers Choset's four opening
questions of Chapter 3 — how much information specifies a robot's position, how to represent it, what
its mathematical properties are, and how workspace obstacles become constraints on it — for the cases
that can be computed exactly, and it opens the collision box that Ch. 2 sealed. The "aha": **a
C-obstacle is not a picture of the workspace obstacle; it is the set of configurations at which a
*function* of the configuration, $R(q)$, meets the obstacle — for a disc an inflation, for a polygon a
Minkowski difference computed vertex by vertex, for an arm a blob on a torus no one can draw by hand.**
The Choset section that gives the forward kinematics (Ch. 2) also gives its differential, the Jacobian,
and the configurations where it loses rank; those belong here because every later planner for Reach
(Chs. 7, 17, 21) must already know that $\det J = L_1 L_2 \sin\theta_2$.

Story line:

1. **Hook** — w4.1 autoplays: a slate block slides across the Workbench; on the unwrapped torus beside
   it an amber blob stretches, splits, and rejoins. The caption asks the reader to guess what it is.
2. **Play (C)** — drag the block; then w4.4: three Rustys, one doorway, $\mathcal{Q}_{free}$ disconnects (C figure 3.5).
3. **Formalize (F)** — configuration, C-space, degrees of freedom; $\mathcal{QO}_i$, $\mathcal{Q}_{free}$,
   free and semifree paths; the disc's C-obstacle as a Minkowski sum; dimension by point counting and
   holonomic constraints; Grübler's formula; Choset's table of example C-spaces.
4. **Algorithm** — the box opened: polygons (F.1), the Type A / Type B contact test (F.2), the star
   algorithm (F.3), $SE(2)$ as stacked slices (F.4), GJK (F.5) — and the arm rasterizer, with Choset's
   warning that a grid is not a proof.
5. **Jacobian** — $\dot x = J(q)\dot q$ for Reach; singular configurations $\sin\theta_2 = 0$; the
   polygon's Jacobian (Example 3.8.2) as the velocity of a point on a moving body.
6. **Implementation (P)** — the `cspace` crate: C-obstacle rasterizer for any `Collision`-checkable
   footprint, star algorithm with brute-force cross-check, `jacobian()` validated against `k`.
7. **Integration lab** — the Workbench's full C-space map from the Ch. 2 scene; a start and goal in
   different amber-free components: "no planner in this book will connect these two."
8. **Close** — what is missing: the topology that makes the torus wrap (Ch. 5); the search (Part II).

## 2. Prerequisites & Position

- **Builds on:** Ch. 1 (the C-space reduction; the piano-mover slices), Ch. 2 (`Reach::fk`, `Footprint`,
  `Scene`, the `Collision` contract — explained here; `T2`, `Pose2`, `R2`), Ch. 3 ($D(x)$ — GJK computes it).
- **Feeds into:** Ch. 5 (the torus and $SE(2)$ as manifolds; $D\varphi$ as the chart-free view of $J$),
  Ch. 7 §4.7.3 (workspace forces lifted through $J^\top$; §4.7.1–4.7.2 use the polygon Jacobian), Ch. 8
  (visibility graphs need explicit polygonal $\mathcal{QO}$), Ch. 10 (trapezoidal decomposition of
  polygonal $\mathcal{Q}_{free}$), Ch. 11 (collision-checking schedules against this chapter's raster),
  Chs. 17–18 (singularities as infinite joint velocities), Ch. 21 (Jacobian rank), Appendix C (the card).
- **Baseline sources:** C §3.1 (b. 40–42; the circular robot's $R(x,y)$; $T^2$ and the cut torus, figures
  3.1–3.3), §3.2 (b. 43–47): §3.2.1 circular mobile robot (figures 3.4–3.5), §3.2.2 two-joint planar arm
  (figures 3.6–3.8; the "thicken the robot" footnote), §3.3 The Dimension of the Configuration Space
  (b. 47–50; holonomic vs. nonholonomic; eq. 3.1 Grübler), §3.7 Example Configuration Spaces (b. 68–69;
  Table 3.1; $T^n \ne S^n$, $SE(2) \ne \mathbb{R}^3$), §3.8 (b. 69–71) **Jacobian and singularities
  only** — Example 3.8.1's $J$ and the $\sin\theta_2 = 0$ remark, Example 3.8.2's polygon Jacobian;
  App. F (b. 499–512): F.1 polygons, F.2 intersection tests (eqs. F.2–F.6; Type A/B contacts; the worked
  tables of figure F.7), F.3 the star algorithm and Minkowski difference (eq. F.7, [C ref. 124]), F.4
  $SE(2)$ obstacles (figure F.8, [C refs. 69, 70]), F.5 GJK (Alg. 23, eqs. F.8–F.14, [C ref. 163]
  Gilbert, Johnson and Keerthi). App. B (b. 477) for Minkowski sum/difference, convexity, star-shaped
  sets. Sister cross-links: Ch. 20 (`https://shannon-dynamics.github.io/ProbabilisticRoboticRust/chapters/ch20-motion-planning`),
  whose derivation F.1 is the one-paragraph version of this chapter; Ch. 13
  (`…/chapters/ch13-occupancy-grids`) for the inflated occupancy grid, the disc's $\mathcal{QO}$ on a
  lattice. Modernization: `parry2d` (Dimforge) for GJK/EPA and shape-casting (verify 0.30 API); `k` and
  `urdf-rs` for the Jacobian cross-check (test-only; verify); Lozano-Pérez's C-space formulation (verify
  whether in Choset's bibliography).

## 3. Foundation (F) — Mathematical Core

Chapter notation table:

| Symbol | Meaning |
|---|---|
| $\operatorname{cl}(\cdot)$, $\operatorname{int}(\cdot)$, $\partial$ | closure, interior, boundary (App. B) |
| $A \oplus B$, $A \ominus B$ | Minkowski sum $\{a + b\}$, Minkowski difference $\{a - b\}$ |
| $M$, $N$, $k$, $n$, $f_i$ | mobility, ambient DOF (3 planar / 6 spatial), links, joints, DOF of joint $i$ |
| $g(q, t) = 0$, $g(q, \dot q, t) = 0$ | holonomic constraint; nonholonomic (velocity) constraint |
| $h(x,y) = ax + by - c$, $h^-$, $h^+$ | a line and its negative/positive half-planes (App. F.1) |
| $r_i$, $E^R_i$, $n^R_i(\theta)$; $o_j$, $E^W_j$, $n^W_j$ | robot vertices, edges, outward normals ($\theta$-dependent); obstacle vertices, edges, normals |
| $f^R_{ij}$, $f^W_{ij}$ | the half-plane constraints of Type A and Type B contacts (eqs. F.4, F.6) |
| $J(q) = \partial\varphi/\partial q$ | the Jacobian of the forward kinematics; $\dot x = J(q)\dot q$ |
| $h_Z(x)$, $s_Z(x)$ | GJK support value and support point of polytope $Z$ in direction $x$ |

**Definitions** (definition boxes):

- **Configuration / C-space / DOF** (C §3.1). A complete specification of the position of every point
  of the robot; the space of all such; its dimension, equivalently the minimum number of parameters.
- **C-obstacle, free space, free and semifree paths** (C §3.2). $\mathcal{QO}_i = \{q : R(q) \cap
  \mathcal{WO}_i \ne \emptyset\}$; $\mathcal{Q}_{free} = \mathcal{Q} \setminus \bigcup_i \mathcal{QO}_i$,
  assumed open; a free path lies in $\mathcal{Q}_{free}$, a semifree path in $\operatorname{cl}(\mathcal{Q}_{free})$.
- **Holonomic vs. nonholonomic constraint** (C §3.3). $g(q,t) = 0$ removes one dimension per
  independent constraint; $g(q, \dot q, t) = 0$ constrains velocities and removes none (Ch. 20).
- **Mobility; redundancy** (C §3.3). Open chains count DOF per joint (R, P, spherical); closed chains use
  Grübler's $M = N(k - n - 1) + \sum_i f_i$ for independent constraints (C figure 3.9: six links, seven
  revolute joints, $M = 3(6 - 7 - 1) + 7 = 1$). More DOF than the task needs is redundant.
- **Convex polygonal region, polygonal region, polygon** (App. F.1). An intersection of finitely many
  negative half-planes; a finite union of those; a closed, simply connected polygonal region.
- **Type A and Type B contacts** (App. F.2). An edge of $R$ containing a vertex of $W$; an edge of $W$
  containing a vertex of $R$ — the only ways convex bodies touch without overlapping (eq. F.2).
- **Singular configuration** (C §3.8). A $q$ where $J(q)$ loses rank: some endpoint velocity is impossible.

**Derivations / theorems:**

1. **The disc's C-obstacle is a Minkowski inflation** (C §3.2.1). *Statement:* for a disc of radius $r$
   translating in the plane, $\mathcal{QO}_i = \mathcal{WO}_i \oplus B_r(0)$ — grow polygons outward and
   walls inward by $r$. *Sketch (3 steps):* $R(q) \cap \mathcal{WO}_i \ne \emptyset$ iff some obstacle
   point is within $r$ of $q$ iff $q \in \mathcal{WO}_i \oplus B_r$; the boundary is edge offsets plus
   vertex arcs. *Collapsible:* $|\mathcal{WO} \oplus B_r| = |\mathcal{WO}| + r\,p + \pi r^2$ for convex
   $\mathcal{WO}$ ($[1,2]^2$, $r = 0.5$: $3.7854$); Choset's warning (b. 44) that "simply growing" is special to the disc.
2. **Dimension by point counting** (C §3.3). *Statement:* a planar rigid body has 3 DOF, a spatial one 6;
   $n$ coordinates with $m$ independent holonomic constraints give $n - m$ DOF. *Sketch (3 steps):* place
   $A$ freely (2 or 3); $B$ is constrained to a circle (sphere) about $A$ — one angle (two) remains; $C$ is
   fixed by two (three) distances up to a discrete choice — zero (one angle about $AB$) remains, and every
   other point follows: $2 + 1 = 3$, $3 + 2 + 1 = 6$. *Collapsible:* Reach by the indirect count ($12 - 10 = 2$).
3. **Contact half-planes characterize collision for convex polygons** (App. F.2). *Statement:* for fixed
   $\theta$, $q \in \mathcal{QO}$ iff $f^R_{ij}(q) \le 0$ for every applicable Type A pair and
   $f^W_{ij}(q) \le 0$ for every applicable Type B pair; a pair is applicable when the negated robot edge
   normal lies between the normals of the obstacle edges adjacent to the vertex (eq. F.3), or
   symmetrically (eq. F.5). *Sketch (4 steps):* at a boundary configuration the bodies touch without
   overlapping (eq. F.2); only Type A/B contacts realize that; each applicable pair gives a supporting
   half-plane of the convex set $\mathcal{QO}(\theta)$ (eqs. F.4, F.6); a convex set is the intersection
   of its supporting half-planes — C's figure F.7 tables show one failing inequality separating the bodies.
   *Collapsible:* nonconvex bodies by convex decomposition; why $n^R_i$ depends on $\theta$ only.
4. **The star algorithm computes $\mathcal{QO}(\theta) = \mathcal{WO} \ominus R(0,0,\theta)$ in linear
   time after sorting** (App. F.3). *Statement:* negate the robot's edge normals, merge-sort them with the
   obstacle's by angle, scan once; each time a negated robot normal falls between adjacent obstacle normals
   emit $o_j - r_i$ and $o_j - r_{i+1}$ (Type A), and symmetrically $o_j - r_i$, $o_{j+1} - r_i$
   (Type B); the emitted points, in scan order, are the C-obstacle's vertices counterclockwise. *Sketch
   (3 steps):* a Type A contact persists while the robot slides its edge along the vertex, from $r_i$ on
   $o_j$ to $r_{i+1}$ on $o_j$ — the extremes are C-obstacle vertices, symmetrically for Type B; angular
   order of normals is the boundary order of a convex polygon; the merged scan visits every applicable
   pair once. *Collapsible:* equality with the hull of all pairwise $o_j - r_i$ (the brute-force oracle).
5. **GJK: distance between convex polytopes as the distance from $A \ominus B$ to the origin** (App. F.5).
   *Statement:* $d(A,B) = \min_{z \in A \ominus B}\|z\|$, the minimizer is unique, and Alg. 23 finds it
   using only support functions, which compose: $h_{A \ominus B}(x) = h_A(x) + h_B(-x)$, $s_{A \ominus B}(x)
   = s_A(x) - s_B(-x)$ (eqs. F.13–F.14). *Sketch (4 steps):* rewrite $\min\|a - b\|$ over the Minkowski
   difference (eq. F.10); the difference of convex sets is convex and the norm strictly convex, so the
   minimizer is unique; keep a simplex of $\le 3$ points and its nearest point $x_k$ to the origin; the
   support point in direction $-x_k$ certifies optimality ($\|x_k\| = h_Z(-x_k)$) or strictly improves the
   simplex. *Collapsible:* $A \ominus B$ is never built; what `parry2d` adds (EPA, BVH, shape-casting).
6. **Reach's Jacobian and its singularities** (C §3.8, Example 3.8.1). *Statement:*
   $J(q) = \begin{bmatrix} -L_1\sin\theta_1 - L_2\sin(\theta_1+\theta_2) & -L_2\sin(\theta_1+\theta_2)
   \\ L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) & L_2\cos(\theta_1+\theta_2)\end{bmatrix}$,
   $\det J = L_1 L_2 \sin\theta_2$, singular exactly when $\sin\theta_2 = 0$ (arm straight or folded).
   *Sketch (3 steps):* differentiate $\varphi$ (Ch. 2) entrywise; expand the determinant with the sine
   difference identity; at $\sin\theta_2 = 0$ the columns are parallel, so the image of $\dot q$ is a line
   — motion along the arm is instantaneously impossible. Choset's numbers: $L_1 = L_2 = 1$, $q = (\pi/4,
   \pi/2)$, $J = \begin{bmatrix}-\sqrt2 & -\sqrt2/2\\ 0 & -\sqrt2/2\end{bmatrix}$, $\dot q = (1,0)^\top
   \Rightarrow \dot x = (-\sqrt2, 0)^\top$, $\det J = 1$. *Collapsible:* Example 3.8.2's polygon Jacobian
   $\begin{bmatrix} 1 & 0 & -r_1\sin q_3 - r_2\cos q_3 \\ 0 & 1 & r_1\cos q_3 - r_2\sin q_3\end{bmatrix}$,
   never rank-deficient; $J^\top$ as the force map (Chs. 7, 17).

**Named algorithms:**

| Name | Signature | Complexity |
|---|---|---|
| `inflate_disc` (C §3.2.1) | `(obstacles, r) -> Vec<Polygon-with-arcs>` | $O(\sum n_i)$ |
| `polygon_intersects` (App. F.2) | `(robot: &Polygon, q: Pose2, obstacle: &Polygon) -> bool` | $O(n_R n_W)$ naive; $O(n_R + n_W)$ with sorted normals |
| `star_algorithm` (App. F.3); `se2_slices` (F.4) | `(robot: &Polygon, theta, obstacle: &Polygon) -> Polygon`; $n_\theta$ slices | $O((n_R + n_W)\log(n_R + n_W))$; brute-force oracle $O(n_R n_W \log)$ |
| `GJK` (C Alg. 23) | `(A, B) -> (dist, a*, b*)` | iterations $\le$ #vertices; delegated to `parry2d` |
| `rasterize_cobstacle` (C §3.2.2) | `(robot, scene: &dyn Collision, grid) -> BitGrid` | $O(\#cells)$ collision queries |
| `jacobian` (C §3.8) | `(reach: &Reach<2>, q: T2) -> Matrix2` | $O(1)$ |

**Numeric micro-example** (the contract with the tests in §5): **the C-obstacle of Reach for one small
square.** $L_1 = L_2 = 1$, zero-thickness links, base at the origin, one obstacle $\mathcal{WO} = [1.4,
1.6] \times [-0.1, 0.1]$ (side 0.2, centered at $(1.5, 0)$). Link 1 has length 1 and the square begins at
$x = 1.4$, so $\mathcal{QO}$ comes from link 2 alone. Hand checks: (a) $q = (0, 0)$ — link 2 runs from
$(1,0)$ to $(2,0)$ through the square's center: **collision**. (b) $q = (0, 13°)$ — at $x = 1.4$ link 2's
height is $0.4\tan 13° = 0.092 < 0.1$: **collision**. (c) $q = (0, 15°)$ — height $0.4\tan 15° = 0.107 >
0.1$: it clears the corner $(1.4, 0.1)$, **free**. So the slice of $\mathcal{QO}$ at $\theta_1 = 0$ is
exactly $|\theta_2| \le \arctan(0.1/0.4) = \arctan\tfrac14 \approx \mathbf{0.2450}$ rad $= 14.04°$ — the
angle the near corner subtends from the elbow. (d) $q = (\pi/4, -\pi/2)$ — elbow $(0.7071, 0.7071)$,
link 2 at absolute angle $-45°$, tip $(\sqrt2, 0) = (1.4142, 0)$ inside the square: **collision**.
(e) $q = (\pi/2, \theta_2)$ — elbow $(0, 1)$; the nearest square point $(1.4, 0.1)$ is at distance
$\sqrt{1.96 + 0.81} = 1.664 > L_2$: **free for every $\theta_2$**. Generalizing (e), the $\theta_1$-extent
is the set of elbow positions within 1 of the square: $(1.4 - \cos\theta_1)^2 + (\sin\theta_1 - 0.1)^2
\le 1 \iff 2.8\cos\theta_1 + 0.2\sin\theta_1 \ge 1.97$, i.e., $|\theta_1| \le \mathbf{0.864}$ rad
($49.5°$). On the $1°$ raster (cells centered at integer degrees) the $\theta_1 = 0°$ row has exactly
$2 \cdot 14 + 1 = \mathbf{29}$ occupied cells of 360, rows with $|\theta_1| \ge 50°$ are empty, and the
blob is one connected amber region that does not wrap.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: *the shadow a body casts in the space of its own positions.* Colors: workspace
obstacles slate, C-obstacles amber hatched (the chapter that defines the amber role), start green and
goal red in both panes, the robot orange. The unwrapped torus is the $[-\pi, \pi]^2$ square with
identified edges marked by matching ticks (the folding itself is Ch. 5's w5.1).

- **Widget w4.1: C-Space Morph** — type: wasm-sim (the book's signature figure). Left: the Workbench
  with Reach and one draggable slate obstacle (the "block"). Right: the unwrapped $T^2$ with the
  rasterized $\mathcal{QO}$ in amber, the current $q$ as an orange dot, and any dragged motion's tip path
  drawn in both panes. **Manipulates:** headline — drag the obstacle; disclosure: obstacle size, link
  lengths, raster resolution (0.5°–4°), link thickness (0 = the micro-example's exact segments), toggle
  the second obstacle ("the post"). **Observes:** as the block nears the base the blob grows a vertical
  band (link 1 collides for all $\theta_2$) and wraps across the seam; two obstacles' blobs merge into a
  wall across $\theta_1$ — $\mathcal{Q}_{free}$ disconnects (C figure 3.7); the micro-example preset
  reproduces the 29-cell row. **Misconception killed:** *"the C-obstacle looks like the obstacle."* It is
  a different shape in a different space; dragging the block one way can move the blob the other.
- **Widget w4.2: Minkowski Studio** — type: interactive animation. Two convex polygons (robot in orange
  with its reference point, obstacle in slate) and, step by step, the star algorithm: negated robot
  normals and obstacle normals on a shared angle dial, the merged scan sweeping, each emitted vertex
  $o_j - r_i$ appearing in amber labeled A or B, the final $\mathcal{QO}(\theta)$ closing. A brute-force
  ghost overlays exactly; a "stack" toggle sweeps $\theta$ and draws the $SE(2)$ C-obstacle as stacked
  slices (C figure F.8). **Manipulates:** headline — the robot's orientation $\theta$; disclosure: drag
  vertices; the triangle-vs-unit-square preset. **Observes:** rotating the robot re-pairs the normals and
  can change the vertex count. **Misconception killed:** *"Minkowski sums are expensive."* For convex
  polygons they are a merge of two sorted lists.
- **Widget w4.3: Jacobian Lens** — type: interactive animation. Reach at a draggable $q$; the unit circle
  of joint velocities mapped through $J(q)$ to an ellipse of tip velocities at the tip; $\det J =
  \sin\theta_2$ printed; the columns of $J$ as arrows. **Manipulates:** headline — $\theta_2$;
  disclosure: $\theta_1$, link lengths, show $J^\top$ (tip force → joint torques, Ch. 7's preview).
  **Observes:** the ellipse collapses to a segment along the arm as $\theta_2 \to 0$ or $\pi$; at
  Choset's $q = (\pi/4, \pi/2)$ the vector $\dot q = (1, 0)$ lands on $(-\sqrt2, 0)$. **Misconception
  killed:** *"a singularity is where the arm breaks."* A direction becomes unavailable, and planners that
  ignore it ask for infinite joint speed.
- **Widget w4.4: Disc Inflation** — type: wasm-sim (light). Three Rustys of radii 0, 0.11, and 0.35
  (headline slider for the third) in one Apartment room with a doorway; beneath each, its inflated
  $\mathcal{QO}$ in amber and the same query. **Observes:** the doorway path survives for the real Rusty
  and disappears for the largest — $\mathcal{Q}_{free}$ split in two; free area shrinks by $r\,p + \pi r^2$
  per convex obstacle. **Misconception killed:** *"inflate by the radius and plan for a point"* is
  **right** — and right *only* for discs; the caption points to w4.2.

Dashboard layout sketch: w4.1 is full width with two equal panes sharing the orange dot, disclosure
beneath the right pane; w4.2 and w4.3 are half-width cards beside App. F.3 and §3.8; w4.4 is a three-up
strip beside §3.2.1; the App. F notation rows stay sticky beside w4.2 during Derivations 3–4.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`Matrix2`, `Point2`, `Isometry2`); `parry2d` 0.30 (`parry2d-f64`) — still
the black box for GJK/EPA and shape-casting, now *explained*; `collide` and `robots` (Ch. 2); `k` +
`urdf-rs` as **test-only** dependencies to validate `jacobian()` against a URDF model of Reach (verify);
`rand` 0.9 `SmallRng` for seeded polygon fields; `proptest` (dev).

**Module plan:** `crates/cspace/src/{lib.rs, polygon.rs, contact.rs, star.rs, slices.rs, raster.rs,
jacobian.rs}`; `examples/{reach_square.rs, workbench_map.rs}`; `tests/{star_vs_brute.rs, jacobian_vs_k.rs}`.
Web mirror: `web/lib/cspace/{polygon.ts, star.ts, raster.ts, jacobian.ts}` with `__checks__.ts` invariants.

**Key types & signatures:**

```rust
use nalgebra::{Matrix2, Point2, Vector2};
use robots::{Reach, T2};
use collide::{Collision, Footprint};

// polygon.rs — App. F.1. Counterclockwise vertices; edge normals derived, never stored stale.
pub struct Polygon { verts: Vec<Point2<f64>> }
impl Polygon {
    pub fn new_ccw(verts: Vec<Point2<f64>>) -> Result<Self, NotConvexOrNotCcw>;
    pub fn edge_normal(&self, i: usize) -> Vector2<f64>;          // outward unit normal of edge i
    pub fn transformed(&self, pose: &geom::Pose2) -> Polygon; }    // r_i(x, y, θ)

// contact.rs — App. F.2. The contact test, written out so the reader sees the box's insides.
pub enum ContactKind { TypeA { robot_edge: usize, obs_vertex: usize }, TypeB { obs_edge: usize, robot_vertex: usize } }
pub fn applicable_contacts(robot: &Polygon, theta: f64, obs: &Polygon) -> Vec<ContactKind>;   // eqs. F.3, F.5
pub fn polygon_intersects(robot: &Polygon, q: &geom::Pose2, obs: &Polygon) -> bool;            // all f ≤ 0

// star.rs — App. F.3. The owned Minkowski artifact; Ch. 8 and Ch. 10 import it.
/// QO(θ) = WO ⊖ R(0,0,θ): merge-scan of obstacle normals and negated robot normals.
pub fn star_algorithm(robot: &Polygon, theta: f64, obs: &Polygon) -> Polygon;
/// Oracle for tests and for w4.2's ghost: convex hull of all o_j − r_i.
pub fn minkowski_bruteforce(robot: &Polygon, theta: f64, obs: &Polygon) -> Polygon;
pub fn inflate_disc(obs: &Polygon, r: f64, arc_segments: usize) -> Polygon;                     // §3.2.1
pub fn se2_slices(robot: &Polygon, obs: &Polygon, n_theta: usize) -> Vec<(f64, Polygon)>;       // App. F.4

// raster.rs — §3.2.2. Any robot that can produce a footprint from a chart point.
pub trait FootprintAt<Q> { fn footprint_at(&self, q: &Q) -> Footprint; }   // impl'd for Reach<2> over T2
pub struct TorusGrid { pub n1: usize, pub n2: usize }                 // cells centered on a uniform grid of [-π, π)²
pub struct BitGrid { pub n1: usize, pub n2: usize, bits: Vec<u64> }   // get(i, j); row_count(i); components_torus()
/// Choset's pixel method. A free cell is not a free square: `thicken` > 0 inflates the footprint so
/// free cells certify free neighborhoods (conservative; resolution complete, never complete).
pub fn rasterize_cobstacle<Q>(robot: &dyn FootprintAt<Q>, scene: &dyn Collision, grid: &TorusGrid,
                              chart: impl Fn(usize, usize) -> Q, thicken: f64) -> BitGrid;

// jacobian.rs — §3.8 (Jacobian and singularities; FK itself lives in robots::Reach).
pub fn jacobian(reach: &Reach<2>, q: &T2) -> Matrix2<f64>;      // det = L1·L2·sin θ2, exposed as det_jacobian
pub fn body_point_jacobian(r: Vector2<f64>, q3: f64) -> nalgebra::Matrix2x3<f64>;   // Example 3.8.2
```

**Worked end-to-end example** (`cargo run -p cspace --example reach_square`; $L_1 = L_2 = 1$, `link_radius = 0`, $1°$ raster):

```text
q=(0°,0°) collision   q=(0°,13°) collision   q=(0°,15°) free
q=(45°,-90°) collision (tip (1.4142, 0.0000) inside)   q=(90°,*) free (nearest 1.664 > L2)
slice θ1=0°: 29 occupied cells; analytic half-width atan(1/4) = 0.2450 rad = 14.04°
θ1 extent: rows 0..±49° occupied, ±50° empty; analytic bound 0.864 rad = 49.5°
components on T²: 1 (does not wrap)
Jacobian at (45°,90°): [[-1.4142, -0.7071],[0, -0.7071]]  det = 1.0000  J·(1,0) = (-1.4142, 0)
```

Tests: `reach_square_samples` asserts the five hand-checked cells; `slice_row_count_is_29` and
`extent_rows` assert the raster counts; `slice_halfwidth_analytic` bisects the exact segment test at
$\theta_1 = 0$ and asserts $\arctan(1/4)$ to $10^{-9}$; `jacobian_matches_choset_example` asserts $J$,
$\det J = 1$, and $\dot x = (-\sqrt2, 0)$ to $10^{-12}$; `jacobian_vs_k` asserts agreement with the `k`
crate on `reach.urdf` at 1,000 seeded $q$ to $10^{-9}$; `jacobian_is_fk_derivative` checks central
differences of `Reach::fk`; `star_equals_bruteforce` is a `proptest` over 500 seeded convex
pairs and orientations; `star_triangle_square` asserts the preset (robot $(0,0),(0.2,0),(0,0.2)$; obstacle
$[1,2]^2$) yields the vertices $(1,0.8),(2,0.8),(2,2),(0.8,2),(0.8,1)$ and area $1.42$; `inflate_area`
asserts $3.7854$; `contact_table_f7` reproduces Choset's figure F.7 tables (seven inequalities hold in
case (a), one fails in (b)); `black_box_agrees` asserts `Scene::intersects` equals `polygon_intersects` on 10,000 seeded poses.

**Runnable artifact:** the examples; in-page w4.1–w4.4 run `web/lib/cspace/` with the same 29, the same
five vertices, and the same $J$ pinned in `__checks__.ts`. The integration lab is `workbench_map`: the
full Workbench (block, post, shelf) rasterized at $0.5°$ with `link_radius = 0.03`, `components_torus()`
printed, a start and goal in different components, and the figure saved as the chapter's closing image.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w4.1 | C-Space Morph | wasm-sim | `cspace::raster` + `robots` + `collide`; `web/lib/cspace/raster.ts` | drag obstacle; size; link lengths/thickness; resolution; second obstacle | the C-obstacle is a function's preimage; wrapping; disconnection |
| w4.2 | Minkowski Studio | interactive animation | `cspace::star` + `se2_slices` (+ brute-force ghost); `web/lib/cspace/star.ts` | $\theta$ slider; drag vertices; presets; stack toggle | star algorithm = merge of sorted normals; Type A/B contacts; $SE(2)$ slices |
| w4.3 | Jacobian Lens | interactive animation | `cspace::jacobian` + `robots::Reach`; `web/lib/cspace/jacobian.ts` | $\theta_2$ slider; $\theta_1$; $J^\top$ toggle | velocity ellipse; $\det J = L_1L_2\sin\theta_2$; singularities |
| w4.4 | Disc Inflation | wasm-sim (light) | `cspace::inflate_disc` + `sim::World` | radius slider | Minkowski inflation; $\mathcal{Q}_{free}$ disconnects |
| — | Sliding block hook | animation (autoplay) | `cspace::raster` replay | none | the blob is the obstacle's shadow in $T^2$ |

## 7. Exercises & Extensions

1. **(F)** Prove that the C-obstacle of a convex translating robot against a convex obstacle is convex
   (C problem 3.4), and that $\mathcal{Q}(\mathcal{WO}_i \cup \mathcal{WO}_j) = \mathcal{QO}_i \cup
   \mathcal{QO}_j$ (C problem 3.5). Why does the second fact let `rasterize_cobstacle` take a whole `Scene`?
2. **(F)** Give $\dim \mathcal{Q}$ for (a) two planar rigid bodies tied by a taut rope, (b) two rigidly
   barred together, (c) a train on tracks with and without wheel angles, (d) a sheet of paper (C problem
   3.2). For (c), state which constraint is nonholonomic.
3. **(C — predict, then verify with w4.1)** Load the micro-example preset and move the square to be
   centered at $(0.95, 0)$. Predict, before releasing: does a vertical band appear (link 1 colliding for
   all $\theta_2$), and with what half-width in $\theta_1$? Verify, then explain why the band's edges are
   *not* at $\pm\arctan(0.1/0.85)$.
4. **(C — w4.3)** At $\theta_2 = 10°$, predict the ratio of the velocity ellipse's axes from $\det J$ and
   $\|J\|$; verify. Then find a $q$ where a unit tip force needs the largest joint torque ($J^\top$ toggle).
5. **(P)** Implement `rasterize_cobstacle` for `Reach<3>` on $T^3$ (a `BitGrid3`) and render three
   $\theta_3$-slices; verify that $\theta_3 = 0$ with $L_3 = 0$ reproduces the `Reach<2>` grid bit for bit.
6. **(P, stretch)** Implement C problem 3.18: read a convex robot and convex obstacles from files, compute
   $\mathcal{QO}(\theta)$ with `star_algorithm` for a chosen $\theta$, union the results, and show (flood
   fill) an orientation at which a translation path exists and one at which it does not. Cross-check the
   union against `rasterize_cobstacle` on $\mathbb{R}^2$.

## 8. Modernization Notes

- **Kept from Choset, and why.** The questions of §3.1–3.3 in Choset's order (configuration, obstacles,
  dimension); the disc and the 2R arm as the running examples, with figures 3.4–3.8 recreated as live
  widgets; point counting and Grübler's formula; Table 3.1; Example 3.8.1's exact numbers; and App. F in
  full, because *delegating* collision geometry to a crate is only honest if the reader has seen the
  algorithms once. §3.2.2's conservative-thickening footnote becomes a parameter.
- **Added beyond 2005.** (i) The micro-example: a C-obstacle with an analytic slice width and extent,
  pinned at three precisions (hand, raster, bisection) — Choset's figures 3.6–3.8 came from an applet and
  carried no numbers. (ii) The star algorithm with a brute-force property test, and `black_box_agrees`,
  which ties the reader's own contact test to `parry2d`'s GJK: the black box is verified, not trusted.
  (iii) `jacobian()` validated against a URDF model through `k` (verify). (iv) `components_torus()` —
  connectivity measured before Ch. 5 states it. (v) What modern engines add to GJK (EPA, BVH, shape-cast).
- **Dropped/condensed.** §3.4–3.6 move to Ch. 5; the FK half of §3.8 lives in Ch. 2. App. F's extension
  of GJK to polyhedra is one sentence (Appendix C). The four-bar linkage (C problem 3.7) becomes a w4.1 preset.
- **Honesty items the prose must keep.** A rasterized C-obstacle is a *picture*, not a certificate: free
  cells do not certify free paths between them unless the footprint is thickened, and even then the
  planner is only resolution complete (C's footnote, b. 47). The star algorithm is for convex pairs;
  nonconvex bodies are decomposed. GJK's uniqueness is of the minimizing *difference* $z^*$, not of the
  witness pair. Singularities are configurations, not failures: "one direction is unavailable," never "the arm cannot move."
