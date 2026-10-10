# Chapter 5 — Configuration Space II: Topology, Manifolds, and Rigid Bodies

> Part I — Foundations: Robots, Worlds, and Configuration Space · Estimated length: 9 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Chapter 4 drew Reach's configuration space as a square with amber blobs and kept saying "the edges are
identified." This chapter says what that means, why it matters to a planner, and how to make the
compiler enforce it. Choset's §3.4–3.5 are the most abstract pages of his book, and the temptation is to
skip them; the reason not to is concrete: every planner from Ch. 6 onward needs a *distance* between
configurations, an *interpolation* between them, and a way to *sample* them — and all three are wrong if
you pretend $T^2$ is $\mathbb{R}^2$ or $SE(2)$ is $\mathbb{R}^3$. The "aha": **a configuration space is
a manifold — local coordinates, usually no global ones — so every planner must be written against the
operations the manifold provides (`dist`, `interpolate`, `sample`) and never against the coordinates.**
That sentence is a Rust trait, this chapter's owned artifact, implemented for $S^1$, $T^2$, $SE(2)$, and
$SO(3)$. Rigid bodies get $SO(n)$ and $SE(n)$ with Choset's three-frame worked example, and the chapter
closes by reading Reach's Jacobian as the differential $D\varphi$, a linear map between tangent spaces
that does not care which chart you used.

Story line:

1. **Hook** — w5.1 autoplays: the $[-\pi, \pi]^2$ square of Ch. 4 folds into a doughnut; a purple path
   that looked like two disconnected pieces on the square becomes one short curve on the torus.
2. **Play (C)** — drag a path across the seam; then w5.3: the four charts of $S^1$ hand a point to each other.
3. **Formalize (F)** — mappings; homeomorphism and diffeomorphism; manifolds, charts, atlases;
   connectedness and compactness; C-spaces that are not manifolds.
4. **Algorithm** — metrics (App. C.2) on $S^1$, $T^2$, $SE(2)$, $SO(3)$; geodesic interpolation; uniform
   sampling; embeddings and the matrix groups $SO(n)$, $SE(n)$ with composition, frame change, displacement.
5. **Rotations in 3D** (short) — Euler angles lose a chart at $R_{33} = \pm 1$; quaternions cover $SO(3)$
   with an atlas; the exponential-coordinates view, cross-linked.
6. **Implementation (P)** — `trait Manifold` and its four implementations; the compile-error demo for
   mixing spaces; property tests for the metric axioms.
7. **Integration lab** — Ch. 4's Workbench raster on $T^2$: components counted with wraparound, the
   shortest seam-crossing path measured with `dist`.
8. **Close** — $D\varphi$ revisited; the forward map to Parts II–III (graphs, samplers, kd-trees generic over `Manifold`).

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (the configuration types `T2`, `Pose2`, `HitchCfg`; the ported `Pose2` is $SE(2)$
  and gets an `impl Manifold`), Ch. 4 (the unwrapped torus; $J$ becomes $D\varphi$), Ch. 1 (paths as maps).
- **Feeds into:** Ch. 6 (graphs embedded in manifolds; edge costs are `dist`), Ch. 7 (gradients on $T^2$
  and $SE(2)$ need tangent spaces), Ch. 11 (kd-tree neighbors under `Manifold::dist`; samplers via
  `Manifold::sample`; `trait Steer`'s default is `interpolate`), Chs. 12–14 (tree planners and composite
  spaces generic over `Manifold`), Ch. 17 ($SO(3)$ inertia), Chs. 20–21 (tangent spaces; $SE(2)$ as the
  car's group), Appendix B (the reference card).
- **Baseline sources:** C §3.4 (b. 50–59): §3.4.1 homeomorphisms and diffeomorphisms (Definitions
  3.4.1–3.4.2; eqs. 3.2–3.5; figures 3.11–3.12; neighborhoods), §3.4.2 differentiable manifolds
  (Definitions 3.4.3–3.4.5; the four-chart atlas of $S^1$; figures 3.13–3.14), §3.4.3 connectedness and
  compactness, §3.4.4 not all configuration spaces are manifolds; §3.5 (b. 59–66) with §3.5.1 matrix
  representations ($SO(n)$, $SE(n)$; the frames $A, B, C$ example with $T_{AB}$, $T_{BC}$, $T_{AC}$ and
  the point $w$; body- vs. world-frame transformations, figures 3.17–3.19); §3.6 Parameterizations of
  $SO(3)$ (b. 66–67; eqs. 3.6–3.8) kept short; App. B (b. 475–477); App. C (b. 478–486): C.1 topology,
  C.2 metric spaces (Definition C.2.1), C.3 normed and inner product spaces, C.4 continuous functions
  (Definition C.4.1, Example C.4.2), C.5 Jacobians and gradients (the chain rule; Example C.5.1); App. E
  (b. 489–498) **demoted**: E.1 Euler angles and the lost chart at $R_{33} = \pm1$, E.3 axis–angle, E.4
  quaternions as a four-chart atlas — one short section here, the card in Appendix B. Sister cross-links:
  Ch. 3 (`https://shannon-dynamics.github.io/ProbabilisticRoboticRust/chapters/ch03-geometry-of-motion`)
  for $SE(2)$ as a Lie group, $\exp/\log$, $\boxplus/\boxminus$, and its Appendix C for $SO(3)$ machinery;
  Ch. 7 (`…/chapters/ch07-ekf-ukf-manifolds`) for why estimators also need manifolds. Modernization: Solà,
  Deray & Atchuthan, micro-Lie theory (arXiv 1812.01537; verify); Lynch & Park, *Modern Robotics* (verify).

## 3. Foundation (F) — Mathematical Core

Chapter notation table:

| Symbol | Meaning |
|---|---|
| $\phi : S \to T$, $\phi(S)$, $\phi^{-1}(T)$ | a mapping; image; preimage |
| $(U, \phi)$; $\psi \circ \phi^{-1}$ | a chart (open $U \subset \mathcal{Q}$, diffeomorphism $\phi$ onto an open set of $\mathbb{R}^k$); a transition map |
| $B_\epsilon(p)$, $\operatorname{nbhd}(p)$; $C^0, C^k, C^\infty$ | open ball; neighborhood; continuous, $k$ times continuously differentiable, smooth |
| $T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}$, $T_{AB}$, $w_A$ | a homogeneous transform; frame $B$ in $A$; the point $w$ in $A$'s coordinates |
| $d_{S^1}$, $d_{T^2}$, $d_{SE(2)}^{w}$, $d_{SO(3)}$; $(\phi, \theta, \psi)$ | the chapter's metrics, $w$ the rotation weight (meters per radian); Z-Y-Z Euler angles (§3.6) |
| $T_q\mathcal{Q}$, $D\varphi_q : T_q\mathcal{Q} \to T_{\varphi(q)}M$ | tangent space (informal here, formal in Ch. 20); the differential |

**Definitions** (definition boxes, in Choset's order):

- **Surjective / injective / bijective; homeomorphism; diffeomorphism** (§3.4.1, Defs. 3.4.1–3.4.2).
  Onto; one-to-one; both. A bijection continuous both ways; a smooth bijection smooth both ways. Every
  diffeomorphism is a homeomorphism, not conversely: the circle, ellipse, and racetrack (eqs. 3.2–3.5)
  are pairwise homeomorphic (radial projection), the circle and ellipse are diffeomorphic, but the
  racetrack's curvature jumps at $(\pm1, \pm1)$, so no diffeomorphism reaches it.
- **Neighborhood; locally diffeomorphic** (§3.4.1). $U \ni p$ is a neighborhood if every point of $U$ has
  an open ball inside $U$; $S$ is locally diffeomorphic to $T$ if each point has a neighborhood
  diffeomorphic to an open set of $T$ — the sphere is locally a plane.
- **Manifold; chart; $C^\infty$-related charts; atlas** (Defs. 3.4.3–3.4.5). A set locally homeomorphic
  to $\mathbb{R}^k$; a pair $(U, \phi)$; two charts whose transition maps are smooth on the overlap; a
  family of pairwise $C^\infty$-related charts covering $\mathcal{Q}$ (its *differentiable structure*).
- **Connected; components; compact** (§3.4.3). A path between any two points; maximal connected subsets
  — different components for $q_{start}$ and $q_{goal}$ means *no plan*; closed and bounded in some
  $\mathbb{R}^n$ — $S^n$, $T^n$, $SO(n)$ are, $\mathbb{R}^n$ and $SE(n)$ are not; products of compact spaces are compact.
- **Metric space; continuous function** (Defs. C.2.1, C.4.1). $d \ge 0$ with definiteness, symmetry, and
  the triangle inequality, whose open balls generate the topology; preimages of open sets are open (eq. C.1).
- **$SO(n)$, $SE(n)$** (§3.5.1). $SO(n) = \{R : RR^\top = I, \det R = 1\}$; $SE(n)$ the matrices
  $\begin{bmatrix} R & p \\ 0 & 1\end{bmatrix}$; groups and smooth submanifolds of $\mathbb{R}^{m^2}$ — Lie groups (C footnote 9).

**Derivations / theorems:**

1. **$S^1$ needs at least two charts; four suffice** (§3.4.2, figure 3.14). *Statement:* no single chart
   covers $S^1$; the charts $U_1 = \{x_2 > 0\}, \phi_1 = x_1$; $U_2 = \{x_2 < 0\}, \phi_2 = x_1$; $U_3 =
   \{x_1 > 0\}, \phi_3 = x_2$; $U_4 = \{x_1 < 0\}, \phi_4 = x_2$, with $\phi_1^{-1}(z) = (z, \sqrt{1 -
   z^2})$ etc., form an atlas. *Sketch (4 steps):* (i) a single chart would be a homeomorphism onto an open
   interval, but removing a point disconnects an interval and not a circle; (ii) the $U_i$ are open and
   cover, with $U_1 \cap U_2 = U_3 \cap U_4 = \emptyset$; (iii) each of the four remaining transition maps
   is $\pm\sqrt{1 - z^2}$ on an open subinterval of $(-1, 1)$, smooth there; (iv) hence $C^\infty$-related.
   *Collapsible:* the two-chart angle atlas (C problem 3.9) that `manifold::S1` uses; Ch. 4's "single
   angle in $[0, 2\pi)$" is a chart with a seam, not an atlas.
2. **Joint limits change the topology** (§3.4.1 b. 55, §3.4.3–3.4.4). *Statement:* $T^2 \not\cong
   \mathbb{R}^2$ (compact vs. not), but a 2R arm with strict joint limits has C-space an open rectangle,
   diffeomorphic to $\mathbb{R}^2$. *Sketch (3 steps):* continuous images of compact sets are compact and
   $\mathbb{R}^2$ is unbounded; with limits each joint ranges over an open interval; a stretched $\tan$ is
   a diffeomorphism onto $\mathbb{R}$, and products preserve the claim. *Collapsible:* closed limits give a
   manifold *with boundary*; the figure-eight C-space of some one-DOF parallel mechanisms is no manifold.
3. **$SE(n)$ composes by subscript cancellation; body vs. world transformations** (§3.5.1). *Statement:*
   $T_{AB}T_{BC} = T_{AC}$; $w_A = T_{AB}w_B$ changes a point's frame; $T_{AB}w_A$ *displaces* it by the
   motion carrying $A$ to $B$; for a transform $T_1$, $T_{AB}T_1$ acts in $B$'s frame (body), $T_1T_{AB}$
   in $A$'s (world), and these differ. *Sketch (4 steps):* block-multiply to get $\begin{bmatrix} R_1R_2 & R_1p_2 + p_1 \\
   0 & 1\end{bmatrix}$; closure and $T^{-1} = \begin{bmatrix} R^\top & -R^\top p \\ 0 & 1\end{bmatrix}$
   make $SE(n)$ a group; the constraint count $9 - 6 = 3$ (unit, orthogonal columns) gives $\dim SO(3)$;
   the orderings differ because multiplication does not commute — body transformations stack on the
   right, world ones on the left. *Collapsible:* Choset's numeric example (figures 3.17–3.19) as `Pose2`
   (the micro-example); C problem 3.23: $SE(2)$ and $\mathbb{R}^2 \times SO(2)$ are homeomorphic, not isomorphic.
4. **The chapter's metrics are metrics, and `interpolate` follows geodesics** (App. C.2). *Statement:*
   $d_{S^1}(a,b) = \min(|a-b|, 2\pi - |a-b|)$; $d_{T^2}$ is the $\ell^2$ sum of two $d_{S^1}$;
   $d^w_{SE(2)} = \sqrt{\Delta x^2 + \Delta y^2 + w^2 d_{S^1}(\theta, \theta')^2}$; $d_{SO(3)}(R, R') =
   \arccos\big((\operatorname{tr}(R^\top R') - 1)/2\big)$ — each satisfies Definition C.2.1, and
   `interpolate(t)` moves along the shortest arc. *Sketch (4 steps):*
   $d_{S^1}$ is the quotient metric of $\mathbb{R}$ by $2\pi\mathbb{Z}$ and inherits the triangle
   inequality; $\ell^2$ combinations of metrics are metrics (Minkowski's inequality); $w$ is the exchange
   rate between meters and radians and has *no canonical value* (Ch. 11 measures its effect on PRM);
   $d_{SO(3)}$ is the axis–angle $\theta$ of $R^\top R'$ (App. E.3). *Collapsible:* the antipodal
   ambiguity ($d_{S^1} = \pi$) and its deterministic tie-break; C.2–C.3 on equivalent metrics and norms.
5. **$D\varphi$ is chart-independent; the Jacobian is its matrix** (App. C.5; §3.8 revisited).
   *Statement:* $D\varphi_q$ is a linear map $T_q\mathcal{Q} \to T_{\varphi(q)}M$ whose matrix in charts
   is $J(q)$; under a chart change $h$ the matrix transforms by the chain rule $D(\varphi \circ h) =
   D\varphi \cdot Dh$, and for the torus's angle charts $Dh = I$, so Ch. 4's $J$ is the same matrix in
   every chart. *Sketch (3 steps):* define the differential along a curve by $\tfrac{d}{dt}\varphi(c(t)) =
   D\varphi\,\dot c$; apply the chain rule (C.5); the transition maps of the angle atlas are translations by
   $2\pi$ with derivative 1. *Collapsible:* Example C.5.1 (polar-to-Cartesian chain rule giving $-4\pi\sin
   2\pi t$) as a worked check; velocities are columns, forces are rows (C b. 486) — why $J^\top$ maps forces.
6. **Euler angles lose a chart; quaternions provide an atlas** (§3.6, App. E.1, E.4 — the short rotation
   section). *Statement:* the Z-Y-Z map $(\phi,\theta,\psi) \mapsto R_{z,\phi}R_{y,\theta}R_{z,\psi}$
   (eqs. 3.6–3.8) is a chart only on $\{R : R_{33} \ne \pm1\}$ — at $R_{33} = 1$ only $\phi + \psi$ is
   determined (eq. E.2); unit quaternions $Q = (\cos\tfrac\theta2, n\sin\tfrac\theta2)$ cover $SO(3)$ with
   four charts $U_i = \{|q_i| \text{ largest}\}$. *Sketch (3 steps):* multiply out eq. 3.7 at $\theta = 0$
   and observe the collapse to a single angle; $\|Q\| = 1$ from $n$ unit (eq. E.27) and $R(Q)$ (eq. E.28)
   is a rotation; the four $U_i$ cover because some component has largest magnitude. *Collapsible:* the
   E.1 inverse (eqs. E.3–E.11), axis–angle (E.18–E.21), quaternion product and conjugate (E.31–E.35) — on
   the Appendix B card — and the cross-link to the sister book's Ch. 3, whose `exp`/`log` `So3` reuses.

**Named algorithms** (this chapter's are operations, not planners):

| Name | Signature | Complexity |
|---|---|---|
| `Manifold::dist` | `(&self, other: &Self) -> f64` | $O(\dim)$ |
| `Manifold::interpolate` | `(&self, other: &Self, t: f64) -> Self` (geodesic, short way) | $O(\dim)$ |
| `Manifold::sample` | `(rng: &mut Rng, bounds: &Self::Bounds) -> Self` (uniform on compact factors) | $O(\dim)$ |
| `Pose2::compose` / `::inverse` / `::act` (ported) | $T_{AB}T_{BC}$; $T^{-1}$; $w_A = T_{AB}w_B$ | $O(1)$ |
| `So3::axis_angle` / `::euler_zyz`; `chart_components` | per App. E; `(bitgrid, wrap) -> usize` on Ch. 4's raster | $O(1)$, `Err(GimbalLock)` at $R_{33} = \pm1$; $O(\#cells)$ |

**Numeric micro-example** (the contract with the tests in §5). **On the circle and torus:** angles
$a = 0.1$ and $b = 6.2$ rad. $|a - b| = 6.1$ and $2\pi - 6.1 = 0.1832$, so $d_{S^1}(a, b) =
\mathbf{0.1832}$ — not $6.1$. The midpoint `interpolate(a, b, 0.5)` goes the short way, *downward* from
$0.1$ by $0.0916$: $\mathbf{0.0084}$ (wrapped), not the chart midpoint $3.15$. On $T^2$ with $p = (0.1,
3.0)$, $q = (6.2, 3.5)$: $d_{T^2} = \sqrt{0.1832^2 + 0.5^2} = \mathbf{0.5325}$. On $SE(2)$ with $w = 0.5$
m/rad, from $(0,0,0)$ to $(1, 0, \pi)$: $d = \sqrt{1 + 0.25\pi^2} = \mathbf{1.8620}$. **Choset's frames,
in the plane** (§3.5.1, figure 3.17 — the planar part of his $4\times4$ matrices): $T_{AB}$ has $R =
\mathrm{rot}(\pi)$, $p = (-2, 0)$, i.e. `Pose2 { x: -2, y: 0, theta: π }`; $T_{BC}$ has $R =
\mathrm{rot}(-\pi/2)$, $p = (-4, -1)$. Then $T_{AC} = T_{AB}T_{BC}$ has $R = \mathrm{rot}(\pi/2)$ and $p =
R_{AB}\,p_{BC} + p_{AB} = (4, 1) + (-2, 0) = (2, 1)$: `Pose2 { x: 2, y: 1, theta: π/2 }`, matching Choset's
$\begin{bmatrix} 0 & -1 & 2 \\ 1 & 0 & 1\end{bmatrix}$. The point $w_C = (-2, 1)$: $w_B = R_{BC}w_C +
p_{BC} = (1, 2) + (-4, -1) = \mathbf{(-3, 1)}$; $w_A = R_{AB}w_B + p_{AB} = (3, -1) + (-2, 0) =
\mathbf{(1, -1)}$ — both as in Choset. Body vs. world with $T_1 = $ `Pose2 { x: 0, y: 1, theta: π }`
(figure 3.19): $T_{AB}T_1 = $ `Pose2 { -2, -1, 0 }` and $T_1T_{AB} = $ `Pose2 { 2, 1, 0 }`. **$D\varphi$
check:** at $q = (\pi/4, \pi/2)$ the matrix of $D\varphi_q$ is Ch. 4's $J$, $D\varphi_q(1, 0) =
(-\sqrt2, 0)$, and shifting $\theta_1$'s chart by $2\pi$ changes nothing.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: *maps of a world that has no single map.* Colors: configuration-space objects as in
Ch. 4 (C-obstacles amber, paths purple, the configuration dot orange); charts are translucent sheets in
the neutral chrome palette with teal edges, since they are not data roles; a chart's seam is dashed teal.

- **Widget w5.1: Torus Unwrapper** — type: interactive animation (flagship). The Ch. 4 square with its
  amber C-obstacles, a draggable orange configuration dot, and a purple path. A fold slider rolls the
  square into a cylinder and then a torus in $\mathbb{R}^3$, carrying obstacles and path along; the
  identified edges visibly meet. **Manipulates:** headline — the fold slider; disclosure: drag the path's
  endpoints across the seam, toggle "naive chart distance" vs. `dist` in the readout, toggle joint limits
  (which cut the torus open into a disc). **Observes:** a path in two pieces on the square is one curve on
  the torus; `dist` is shorter than the chart distance exactly when the short way crosses a seam; with
  joint limits on, no path may cross. **Misconception killed:** *"$T^2$ is a square with periodic
  boundary conditions."* The square is one chart; the torus is the space.
- **Widget w5.2: Rotation Zoo** — type: interactive animation. One rigid body (a toy Hitch in 3D, the
  only 3D rendering in Part I) performs a recorded tumbling motion; three readouts track it as time
  series: Z-Y-Z Euler angles, axis–angle, and a unit quaternion. **Manipulates:** headline — a scrubber;
  disclosure: steer the motion through $R_{33} = 1$, choose another Euler convention. **Observes:** as the
  body passes $\theta = 0$ the curves for $\phi$ and $\psi$ swing wildly while their sum stays smooth (eq.
  E.2 live); the quaternion stays smooth. **Misconception killed:** *"gimbal lock is mechanical."* It is a missing chart.
- **Widget w5.3: Atlas of the Circle** — type: interactive animation. $S^1$ in the plane with Choset's
  four chart domains as colored arcs; below, four number lines showing each chart's coordinate of the
  current point (blank outside its domain); a moving point hands off between charts and the transition
  map $\pm\sqrt{1-z^2}$ is plotted on the overlap. **Manipulates:** headline — drag the point;
  disclosure: the two-chart (angle) atlas; the "single chart" attempt and its seam. **Observes:** every
  point lies in some chart, transitions are smooth on overlaps, the single-chart attempt tears.
  **Misconception killed:** *"an angle is a number."* An angle is a point on $S^1$; a number is its chart coordinate.
- **Widget w5.4: Frame Composer** — type: interactive animation. Choset's frames $A, B, C$ and the point
  $w$ on a unit grid (figure 3.17), with draggable frames. Live: $T_{AB}$, $T_{BC}$, their product, and
  $w$ in each frame, with subscript cancellation animated as matching subscripts dim out. A toggle applies
  $T_1$ on the right (body) or left (world) and draws $B'$ vs. $B''$ (figure 3.19). **Manipulates:**
  headline — drag frame $B$; disclosure: edit $T_1$, show the matrices. **Observes:** the micro-example's
  numbers; the two placements differ. **Misconception killed:** *"a transform is a position plus an
  angle."* It is a group element, and the order of multiplication is the whole content.

Dashboard layout sketch: w5.1 full width at the top of the C section, fold slider under the canvas, dual
distance readout at right; w5.3 and w5.4 half-width beside §3.4.2 and §3.5.1; w5.2 beside the rotation section.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`UnitQuaternion`, `UnitComplex`, `SVector`); `rand` 0.9 `SmallRng` through
the ported `Rng` for `sample`; `core::geom::Pose2` (ported $SE(2)$, Ch. 2) — `Manifold` is implemented
*for* it, the type is not redefined; `cspace` (Ch. 4) for the integration lab; `proptest` and `trybuild` (dev).

**Module plan:** `crates/manifold/src/{lib.rs, s1.rs, torus.rs, se2.rs, so3.rs, product.rs}`;
`examples/{seams.rs, frames.rs}`; `tests/{metric_axioms.rs, choset_frames.rs, ui/mix_spaces.rs}`. Web
mirror: `web/lib/manifold/{s1.ts, torus.ts, se2.ts, so3.ts}`, pinned in `__checks__.ts`.

**Key types & signatures:**

```rust
use core::{geom::Pose2, prob::Rng};
use nalgebra::{SVector, UnitQuaternion};

/// THE OWNED ARTIFACT. Every planner from Ch. 6 on is generic over this trait and never
/// touches coordinates directly. `DIM` is the manifold dimension, not the embedding dimension.
pub trait Manifold: Copy + PartialEq + core::fmt::Debug {
    const DIM: usize;
    type Bounds;                                        // sampling box for noncompact factors; () otherwise
    fn dist(&self, other: &Self) -> f64;                // a metric (App. C.2) — property-tested
    fn interpolate(&self, other: &Self, t: f64) -> Self; // geodesic, t ∈ [0, 1], the short way
    fn sample(rng: &mut Rng, bounds: &Self::Bounds) -> Self; // uniform w.r.t. the natural measure
}

/// S¹ as a wrapped angle. The chart is (-π, π]; the type guarantees the representative.
#[derive(Clone, Copy, Debug, PartialEq)] pub struct S1(f64);   // S1::new wraps; S1::angle reads
impl Manifold for S1 {
    const DIM: usize = 1; type Bounds = ();
    fn dist(&self, o: &Self) -> f64 { let d = (self.0 - o.0).abs(); d.min(core::f64::consts::TAU - d) }
    fn interpolate(&self, o: &Self, t: f64) -> Self { S1::new(self.0 + t * S1::new(o.0 - self.0).0) } // short way
    fn sample(rng: &mut Rng, _: &()) -> Self { S1::new(rng.uniform(-core::f64::consts::PI, core::f64::consts::PI)) }
}

/// Tⁿ as a product (ℓ² of S1::dist); Reach's T2 (Ch. 2) is `Torus<2>` by a `From` impl.
#[derive(Clone, Copy, Debug, PartialEq)] pub struct Torus<const N: usize>(pub [S1; N]);

/// SE(2) is the ported Pose2; the metric needs an exchange rate between meters and radians.
#[derive(Clone, Copy, Debug, PartialEq)] pub struct Se2 { pub pose: Pose2, pub w: f64 }   // w: m/rad, no canonical value
impl Manifold for Se2 {
    const DIM: usize = 3; type Bounds = Aabb;   // sampling box for (x, y); θ uniform
    fn dist(&self, o: &Self) -> f64 { /* sqrt(dx² + dy² + w² · S1::dist²) */ }
    fn interpolate(&self, o: &Self, t: f64) -> Self { /* straight in (x,y), short way in θ */ }
    fn sample(rng: &mut Rng, b: &Aabb) -> Self { /* uniform box × uniform S¹ */ } }

/// SO(3) stored as a unit quaternion (App. E.4); the metric is the axis–angle θ of R⁻¹R' (App. E.3).
#[derive(Clone, Copy, Debug, PartialEq)] pub struct So3(pub UnitQuaternion<f64>);
impl So3 { pub fn euler_zyz(&self) -> Result<[f64; 3], GimbalLock>;   // Err iff |R33| == 1 (eq. E.2)
           pub fn axis_angle(&self) -> (SVector<f64, 3>, f64); }       // eqs. E.19–E.21; θ ∈ [0, π]
impl Manifold for So3 { const DIM: usize = 3; type Bounds = (); /* dist = 2·acos(|⟨q, q'⟩|); slerp; Haar-uniform */ }

pub struct Product<A: Manifold, B: Manifold>(pub A, pub B);   // SE(2) × S¹ for Hitch; Ch. 14's composites

/// The first generic planner-shaped function in the book: a geodesic local path.
pub fn geodesic_path<M: Manifold>(a: &M, b: &M, n: usize) -> Vec<M> {
    (0..=n).map(|i| a.interpolate(b, i as f64 / n as f64)).collect()
}
```

The deliberate compile error (`tests/ui/mix_spaces.rs`, checked by `trybuild`):
`geodesic_path(&Torus::<2>::from(q_reach), &rusty_pose_as_se2, 10)` fails with "expected `&Torus<2>`,
found `&Se2`" — printed under the heading *the type system knows that $T^2 \ne SE(2)$* (Choset's §3.7 list).

**Worked end-to-end example** (`cargo run -p manifold --example seams`) prints
`S1 dist(0.1, 6.2) = 0.1832  chart |a-b| = 6.1000  midpoint = 0.0084 (chart midpoint 3.1500)`,
`T2 dist = 0.5325`, `SE2 (w=0.5) dist((0,0,0),(1,0,π)) = 1.8620`; `examples/frames` prints
`T_AC = Pose2 { 2, 1, 1.5708 }  w_B = (-3, 1)  w_A = (1, -1)  body: (-2,-1,0)  world: (2,1,0)  Dφ·(1,0) = (-1.4142, 0)`. Tests: `s1_seam_distance`,
`s1_midpoint_short_way`, `t2_distance`, `se2_weighted_distance` assert the four numbers (closed forms to
$10^{-12}$); `choset_frames` asserts $T_{AC}$, $w_B$, $w_A$, and the body/world results exactly;
`metric_axioms` is a `proptest` asserting definiteness, symmetry, and the triangle inequality for 10,000
seeded triples on each of `S1`, `Torus<2>`, `Se2`, `So3`; `interpolate_geodesic` asserts the endpoints
and `dist(a, interpolate(t)) == t·dist(a, b)` to $10^{-9}$; `euler_gimbal_lock` asserts `Err(GimbalLock)`
at $R_{33} = 1$ and roundtrip elsewhere; `so3_dist_is_angle` asserts `dist` equals the axis–angle of
$R^\top R'$; `dphi_chart_independent` asserts `cspace::jacobian` is unchanged under a $2\pi$ chart shift;
`torus_components_wrap` reruns Ch. 4's Workbench raster with wraparound on and off and asserts the
component count can only *drop* when wrapping is on.

**Runnable artifact:** the examples; in-page w5.1–w5.4 run `web/lib/manifold/`, pinned by the same four
distances and Choset's frames in `__checks__.ts`. The integration lab is `torus_components_wrap` made
visible: $k$ components on the square, $k' \le k$ on the torus, the seam-crossing connection drawn in purple with its `dist`.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w5.1 | Torus Unwrapper | interactive animation | `manifold::Torus` + `cspace::raster` (Ch. 4); `web/lib/manifold/torus.ts` | fold slider; drag path ends; naive-vs-`dist` toggle; joint limits | the square is a chart; seams are artifacts; limits change topology |
| w5.2 | Rotation Zoo | interactive animation | `manifold::So3`; `web/lib/manifold/so3.ts` | scrubber; steer through $R_{33}=1$; convention picker | gimbal lock is a missing chart; quaternions stay smooth |
| w5.3 | Atlas of the Circle | interactive animation | `manifold::S1`; `web/lib/manifold/s1.ts` | drag point; two- vs four-chart atlas; single-chart attempt | charts, overlaps, transition maps, cover |
| w5.4 | Frame Composer | interactive animation | `core::geom::Pose2` (ported) + `manifold::Se2` | drag frames; body/world toggle; show matrices | $T_{AB}T_{BC} = T_{AC}$; frame change vs. displacement; non-commutativity |
| — | Folding hook | animation (autoplay) | `web/lib/manifold/torus.ts` | none | two path pieces are one curve (chapter hook) |

## 7. Exercises & Extensions

1. **(F)** Find two charts for $S^1$ and prove they form an atlas (C problem 3.9); explain why latitude–
   longitude is not a global chart on $S^2$ and exhibit a two-chart atlas (C problem 3.10). Which chart does `manifold::S1` use?
2. **(F)** Show that $SE(2)$ and $\mathbb{R}^2 \times SO(2)$ with $(x_1, R_1)(x_2, R_2) = (x_1 + x_2,
   R_1R_2)$ are homeomorphic but not isomorphic as groups (C problem 3.23): which is commutative, which is `Pose2::compose`?
3. **(C — predict, then verify with w5.1)** Endpoints $(\theta_1, \theta_2) = (-3.0, 0)$ and $(3.0, 0)$:
   predict `dist` (short way vs. chart) and where the purple path crosses the seam; verify; then turn on
   joint limits $|\theta_1| < 3.1$ and explain what happened to the component count.
4. **(C — w5.2)** Scrub to the frame where $R_{33}$ is closest to 1 and read $\phi$ and $\psi$. Predict
   their sum from the quaternion readout via eq. (E.9); verify; then say why commanding $\dot\phi$ and
   $\dot\psi$ separately misbehaves there.
5. **(P)** Implement `Manifold` for the sphere $S^2$ (unit vectors; great-circle `dist`; slerp; uniform
   `sample`). Property-test the axioms and the geodesic property, and show with a test that `Torus<2>`
   and your `S2` disagree on `dist` for identical-looking coordinates — $T^2 \ne S^2$ as a failing assertion.
6. **(P, stretch)** Add exponential coordinates to `Se2` and `So3` — `exp`/`log` reusing the sister
   book's `Pose2` and `nalgebra`'s quaternion log — and implement `interpolate` as $a \cdot \exp(t
   \log(a^{-1}b))$. Show in a test that it coincides with the chart-based interpolation on $SE(2)$ only for
   a matching $w$, and report the maximum discrepancy over 1,000 seeded pairs otherwise (Solà et al., verify).

## 8. Modernization Notes

- **Kept from Choset, and why.** §3.4's definitions in Choset's order with his examples (circle/ellipse/
  racetrack, the four-chart atlas, the "flat world" sphere remark), because the examples are the
  pedagogy; §3.5.1's matrix groups with the complete three-frame numeric example, the one place in Choset
  where $SE(n)$ can be checked by hand, pinned by a test; §3.4.4's warning that not all C-spaces are
  manifolds; App. C.2's metric definition as the contract for `dist`; App. C.5's chain rule for $D\varphi$.
- **Added beyond 2005.** (i) `trait Manifold` — the operations a planner needs, abstracted once and
  implemented four times, with metric axioms property-tested; Choset needs metrics in his Ch. 7 but has
  no reusable abstraction. (ii) The compile-fail test as a pedagogical device: $T^2 \ne SE(2)$ as a type
  error. (iii) The seam micro-example with the wrong chart answers printed beside the right ones.
  (iv) The exponential-coordinates view (Solà, Deray & Atchuthan, verify; Lynch & Park, verify) as a
  cross-link, not a re-derivation. (v) `torus_components_wrap`, which turns §3.4.3's connectedness into a
  measured quantity on the Ch. 4 raster.
- **Dropped/condensed.** §3.6 and App. E are compressed to one section and one widget: the lost chart of
  Euler angles and the quaternion atlas are kept; the inverse formulas (E.3–E.11), roll–pitch–yaw (E.2),
  and the axis–angle derivation (E.13–E.18) move to the Appendix B card. App. C.3's Banach and Hilbert spaces are a pointer.
- **Honesty items the prose must keep.** $SE(2)$'s metric depends on a weight $w$ with no canonical
  value, and planners' behavior (Ch. 11) depends on it — the text never hides $w$ in a default.
  `interpolate` is undefined at antipodal points on $S^1$ and $SO(3)$; the tie-break is deterministic and
  documented. Uniform sampling on $SO(3)$ is Haar-uniform (random unit quaternions), *not* uniform in
  Euler angles — Ch. 11's samplers depend on the distinction. "Manifold" in the trait name is a promise
  about the *operations*, not a proof that every implementing type is a manifold; Choset's warning stands.
