# Chapter 17 — Robot Dynamics

> Part V — Dynamics, Trajectories, and Constraints · Estimated length: 9 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

Parts I–IV planned *paths*: curves in $\mathcal{Q}_{free}$ with no clock attached. A real motor does not
execute a curve; it applies a torque, and the configuration that results depends on masses, lever arms,
and how fast everything else is already moving. This chapter builds the model that turns geometry into
motion for the rest of Part V. The "aha": **the entire dynamics of an arm is encoded in one
configuration-dependent symmetric matrix $M(q)$** — kinetic energy is $\tfrac12 \dot q^\top M(q)\dot q$,
the Coriolis and centrifugal forces are nothing but derivatives of $M$ (the Christoffel symbols), and
gravity is the gradient of a scalar. The reader who has watched Reach's Jacobian ellipse collapse in
Ch. 4 now watches a second ellipse — the inertia ellipse — breathe as the elbow folds, and learns why
"force equals mass times acceleration" is false, term by term, in joint coordinates.

Story line:
1. **Hook** — a Ch. 12 path replayed on Reach at 1× and 2× with a torque gauge; at 2× the gauge pegs
   red and the arm leaves the path: the planner never asked what the motors can do.
2. **Play (C)** — w17.1 Lagrangian Lab: drop Reach from a configuration, watch it swing; energy bars
   stay level. Switch the Coriolis terms off and watch energy drift — the physics is now lying.
3. **Formalize (F)** — generalized coordinates and forces; the Lagrangian $L = K - V$; Euler–Lagrange;
   the standard form $M\ddot q + C\dot q + g = \tau$; Christoffel symbols; why neither $M\ddot q$ nor
   $C\dot q$ is a force on its own.
4. **Algorithm** — the Lagrange recipe as a procedure: link kinetic energies through center-of-mass
   Jacobians, assemble $M$, differentiate for $\Gamma$, read off $g$; the constrained variant with
   multipliers and the projection $P$; Euler's equations for a spinning rigid body.
5. **Implementation (P)** — the `dynamics` crate: `trait Dynamics`, the hand-derived 2R model, the
   Jacobian-assembled 3R model, finite-difference Christoffel cross-check, RK4 integrator, `rapier2d`
   as an external referee in tests.
6. **Integration lab** — Reach under gravity with a PD hold at a planned waypoint; raise the gain, watch
   the overshoot, read the torque Ch. 18 must respect. **Close:** a path $q(s)$ plus these equations is
   a one-dimensional dynamics in $s$ — Ch. 18 opens exactly there.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (Reach's forward kinematics $\varphi$, the Workbench, the three-robot sim), Ch. 4
  (Reach's Jacobian $J(q)$ and its singularities — the center-of-mass Jacobians used here are the same
  construction applied to interior points of the links), Ch. 5 ($SO(3)$ as a matrix group; the
  `Manifold` impl for $SO(3)$ carries the body-frame integrator), Ch. 7 §4.7 (the principle of virtual
  work $u = J^\top f$, re-used verbatim as Choset's feedback transformation $T(q) = J^\top(q)$).
- **Feeds into:** Ch. 18 (path-constrained dynamics $a(s)\ddot s + b(s)\dot s^2 + c(s) = \tau$ is this
  chapter's model restricted to a curve), Ch. 19 (iLQR linearizes `Dynamics::forward` along a
  trajectory), Ch. 20 (nonholonomic mechanical systems reuse the Pfaffian machinery and the geometric
  reading of $\Gamma$), Ch. 21 (kinodynamic planning propagates these equations), Ch. 23 (capstone).
- **Baseline sources:** C §10.1 (b. 349–353: Lagrangian dynamics, the planar body and RP-arm examples),
  §10.2 (b. 353–357: standard form, Christoffel symbols, $T(q)f$ actuation, friction, the inertia matrix
  from kinetic energy), §10.3 (b. 357–361: Pfaffian constraints, Lagrange multipliers, projections
  $P_u$ and $P$, the knife-edge example, holonomic vs. nonholonomic), §10.4 (b. 361–370: mass and
  center of mass, 10.4.1 planar rotation and the parallel-axis theorem, 10.4.2 spatial rotation, Euler's
  equation in spatial and body frames, principal axes, composite bodies, Euler angles revisited);
  background C §3.8 (b. 69–71) and §4.7 (b. 99–104). Our 2R arm is C Problem 10.2 (figure 10.8) worked
  in full. Sister book: Ch. 3 (`ch03-geometry-of-motion`, $SO(3)$), Ch. 4 (`ch04-rusty-and-sensors`,
  the no-slip constraint §3 revisits as Pfaffian), Ch. 9 (`ch09-motion-models`). Modernization set:
  none required; the $\dot M - 2C$ skew-symmetry is classical (Murray, Li & Sastry 1994; Spong,
  Hutchinson & Vidyasagar 2006).

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; $M$, $C$, $g$, $\tau$, $\omega(q)\dot q = 0$ are book-wide):

| Symbol | Meaning |
|---|---|
| $L(q,\dot q) = K(q,\dot q) - V(q)$ | Lagrangian; kinetic minus potential energy |
| $u \in \mathbb{R}^{n}$ | generalized force on $q$; for Reach $u = \tau$ (joint torques). Chs. 18–19 keep $u$ for controls |
| $\Gamma^i_{jk}(q)$, $\Gamma^i(q)$, $\dot q^\top \Gamma(q) \dot q$ | Christoffel symbols of $M$; the $i$-th symmetric $n\times n$ slice; the velocity-product vector |
| $T(q)f = u$, $b(q,\dot q)$ | actuation matrix and raw actuator forces; dissipative generalized force |
| $A(q)$, $\lambda$ | $k \times n$ Pfaffian constraint matrix (rows $\omega_j(q)$); Lagrange multipliers |
| $P_u$, $P$ | projections onto forces that do work / motions that satisfy the constraints |
| $r_{cm}$, $I_{zz}$, $\mathcal{I}$, $\mathcal{I}_s$ | center of mass; planar inertia; body-frame and spatial inertia tensors |
| $\omega$, $\hat\omega$, $R \in SO(3)$ | body angular velocity, its skew matrix, orientation |
| $E = K + V$, $h$ | total mechanical energy; integrator step |

**Definitions** (each a display box):
- *Generalized coordinates and forces*: $q \in \mathbb{R}^n$ (locally) and $u$ such that $u^\top \dot q$
  is power — joint angles and torques for Reach; $(x, y, \theta)$ and (force, force, torque) for a body.
- *Lagrangian*: $L = K - V$ with $K(q,\dot q) = \tfrac12 \dot q^\top M(q) \dot q$ and $V = V(q)$.
- *Inertia matrix*: $M(q) = \partial^2 K / \partial \dot q\, \partial \dot q$; symmetric and positive
  definite because kinetic energy is positive for every nonzero $\dot q$.
- *Christoffel symbols*: $\Gamma^i_{jk} = \tfrac12\big(\partial M_{ij}/\partial q_k + \partial
  M_{ik}/\partial q_j - \partial M_{jk}/\partial q_i\big)$; centrifugal when $j = k$, Coriolis otherwise.
- *Pfaffian constraint*: $A(q)\dot q = 0$ with $k$ independent rows; *holonomic* if each row is
  $\partial c/\partial q$ for some $c(q) = 0$, *nonholonomic* otherwise (Ch. 20 decides which).
- *Inertia tensor*: $\mathcal{I} = \int_V \rho(r)\,(\|r\|^2 \mathbb{1} - r r^\top)\,dV$ in a frame at the
  center of mass; principal axes are its eigenvectors.

**Derivations** (statement → sketch → collapsible):

1. **Euler–Lagrange gives the standard form.** *Statement:* for $L = \tfrac12 \dot q^\top M(q)\dot q -
   V(q)$, $\tfrac{d}{dt}\tfrac{\partial L}{\partial \dot q} - \tfrac{\partial L}{\partial q} = u$ equals
   $M(q)\ddot q + C(q,\dot q)\dot q + g(q) = u$ with $C_{ij} = \sum_k \Gamma^k_{ij}\dot q_k$, $g =
   \partial V/\partial q$. *Sketch (5 steps):* (i) $\partial L/\partial \dot q = M\dot q$; (ii) its time
   derivative is $M\ddot q + \dot M \dot q$ with $\dot M_{ij} = \sum_k (\partial M_{ij}/\partial q_k)
   \dot q_k$; (iii) $\partial L/\partial q_i = \tfrac12 \dot q^\top (\partial M/\partial q_i)\dot q -
   \partial V/\partial q_i$; (iv) subtract and symmetrize the quadratic form in $\dot q$ — the
   symmetrization is exactly what produces the three-term Christoffel formula; (v) read off $g$.
   *Collapsible:* the index gymnastics in full; the RP-arm check (C eqs. 10.5–10.6, symbols
   $\Gamma^1_{12} = \Gamma^1_{21} = m_2 q_2$, $\Gamma^2_{11} = -m_2 q_2$); $C$ is not unique, $C\dot q$ is.
2. **$\dot M - 2C$ is skew-symmetric, hence energy is conserved exactly.** *Statement:* with the
   Christoffel choice of $C$, $\dot q^\top(\dot M - 2C)\dot q = 0$ and therefore $\dot E = \dot q^\top
   (u - b)$. *Sketch (4 steps):* write $(\dot M - 2C)_{ij}$ with the Christoffel formula; the
   $\partial M_{ij}/\partial q_k$ terms cancel; what remains is antisymmetric in $(i,j)$; contract with
   $\dot q \dot q^\top$ to get zero; then $\dot E = \dot q^\top M \ddot q + \tfrac12 \dot q^\top \dot M
   \dot q + \dot q^\top g$ and substitute the equations of motion. *Collapsible:* the full computation
   and why this is the test the `dynamics` crate runs (zero torque, zero friction ⇒ $E$ constant), and
   why dropping $C$ breaks it (w17.1's lie).
3. **The 2R arm (C Problem 10.2).** *Statement:* for links of length $L_i$, mass $m_i$, inertia $I_i$
   about the center of mass at distance $r_i$ from joint $i$, with gravity $a_g$ along $-x_2$,
   $M_{11} = I_1 + I_2 + m_1 r_1^2 + m_2(L_1^2 + r_2^2 + 2 L_1 r_2 \cos q_2)$, $M_{12} = M_{22} + m_2 L_1
   r_2 \cos q_2$, $M_{22} = I_2 + m_2 r_2^2$; with $h = m_2 L_1 r_2 \sin q_2$ the velocity-product
   vector is $(-h\dot q_2^2 - 2h\dot q_1\dot q_2,\; h\dot q_1^2)^\top$ and $g_1 = (m_1 r_1 + m_2 L_1)
   a_g \cos q_1 + m_2 r_2 a_g\cos(q_1{+}q_2)$, $g_2 = m_2 r_2 a_g \cos(q_1{+}q_2)$. *Sketch (5
   steps):* center-of-mass positions by the Ch. 2 forward kinematics applied at $r_i$ instead of $L_i$;
   their velocities via center-of-mass Jacobians $J_{c_i}$; $K = \sum_i \tfrac12(m_i \|J_{c_i}\dot q\|^2
   + I_i \omega_i^2)$ with $\omega_1 = \dot q_1$, $\omega_2 = \dot q_1 + \dot q_2$; hence $M = \sum_i
   (m_i J_{c_i}^\top J_{c_i} + I_i J_{\omega_i}^\top J_{\omega_i})$; differentiate for $\Gamma$ — the
   only nonzero symbols are $\Gamma^1_{12} = \Gamma^1_{21} = \Gamma^1_{22} = -h$, $\Gamma^2_{11} = h$
   (C Problem 10.3). *Collapsible:* every partial derivative; the same recipe for the 3R variant (nine
   $M$ entries, 27 symbols, all from three Jacobians — the pattern `reach3r.rs` implements).
4. **Constrained dynamics and the projection $P$.** *Statement (C eqs. 10.16–10.20):* with $A(q)\dot q
   = 0$, $M\ddot q + C\dot q + g = u + A^\top\lambda$ and $A\ddot q + \dot A\dot q = 0$; eliminating
   $\lambda = (AM^{-1}A^\top)^{-1}\big(-\dot A\dot q + AM^{-1}(C\dot q + g - u)\big)$ gives $P\ddot q =
   PM^{-1}(u - C\dot q - g)$ with $P = \mathbb{1} - M^{-1}A^\top(AM^{-1}A^\top)^{-1}A$ of rank $n - k$.
   *Sketch (5 steps):* differentiate the constraint; solve the force balance for $\ddot q$; substitute;
   solve for $\lambda$; substitute back and factor. *Collapsible:* $P_u = \mathbb{1} - A^\top (AM^{-1}
   A^\top)^{-1}AM^{-1} = P^\top$, orthogonality *with respect to $M$*; the knife-edge (C Example 10.3.1)
   — Rusty's axle, $\dot q_1 \sin q_3 - \dot q_2 \cos q_3 = 0$ — with its closed forms (C 10.25–10.28).
5. **Euler's equation in the body frame.** *Statement (C eqs. 10.43–10.44):* $\dot R = R\hat\omega$,
   $\tau = \hat\omega\,\mathcal{I}\omega + \mathcal{I}\dot\omega$, with $\mathcal{I}$ constant.
   *Sketch (6 steps):* $K = \tfrac12\int \rho\,\|\omega_s \times r_s\|^2 dV = \tfrac12 \omega_s^\top
   \mathcal{I}_s \omega_s$; angular momentum $\mathcal{I}_s\omega_s$ and $\dot{\mathcal{I}}_s = \omega_s
   \times \mathcal{I}_s$ give the spatial form $\tau_s = \hat\omega_s\mathcal{I}_s\omega_s +
   \mathcal{I}_s\dot\omega_s$; substitute $\omega_s = R\omega$, $\tau_s = R\tau$, $\mathcal{I}_s =
   R\mathcal{I}R^\top$, $\hat\omega_s = \dot R R^\top$; use $\hat\omega\omega = 0$; premultiply by
   $R^\top$. *Collapsible:* the principal-axis form (C 10.45), the parallel-axis theorem (C 10.33,
   10.46), and why $\dot\omega \ne 0$ even when $\tau = 0$ — the tumbling w17.3 shows.
6. **RK4 on the first-order state and its energy drift.** *Statement:* with $x = (q, \dot q)$ and $\dot
   x = (\dot q,\; M^{-1}(u - C\dot q - g - b))$, classical RK4 has local error $O(h^5)$ and, for the
   conservative Reach, an energy error that stays bounded by $c\,h^4$ over the simulation horizons we
   use. *Sketch (3 steps):* RK4's Butcher tableau; energy inherits the global $O(h^4)$ error; the test
   bound is calibrated empirically, not a symplectic guarantee. *Collapsible:* why semi-implicit Euler
   drifts linearly, why it stays as the cheap option behind a disclosure, the drift table the test pins.

**Named algorithms** (Choset numbers none of these; names follow his text):

| Algorithm | Signature | Complexity |
|---|---|---|
| Lagrange recipe (C §10.1) | `(links, FK, grav) -> (M(q), Γ(q), g(q))` | $O(n^2)$ Jacobian products for $M$; $O(n^3)$ symbols |
| `christoffel` | `(M: Fn(q) -> Matrix, q) -> Γ` | $O(n^3)$ from $\partial M/\partial q$ (closed form or central differences) |
| `forward_dynamics` (C eq. 11.46 form) | `(q, q̇, u) -> q̈` | one $n \times n$ Cholesky solve, $O(n^3)$ |
| `constrained_forward_dynamics` (C 10.16–10.17) | `(q, q̇, u, A, Ȧ) -> (q̈, λ)` | one $(n{+}k)\times(n{+}k)$ KKT solve |
| `jt_actuation` (C eq. 10.12) | `(J(q), f) -> u` | $O(n\, m)$ |
| `rk4_step` | `(f, x, u, h) -> x` | 4 dynamics evaluations |
| `euler_body_step` (C 10.43–10.44) | `(R, ω, τ, 𝓘, h) -> (R, ω)` | $O(1)$; $R \leftarrow R\exp(h\hat\omega)$ via Ch. 5 `SO3` |

**Numeric micro-example** (the contract with the unit test). Reach with $L_1 = L_2 = 1$ m, uniform rods
so $r_1 = r_2 = 0.5$ m, $m_1 = 2$ kg, $m_2 = 1$ kg, $I_1 = m_1 L_1^2/12 = 1/6$, $I_2 = 1/12$ kg·m²,
$a_g = 9.81$ m/s², at $q = (\pi/4, \pi/2)$ — the configuration of C Example 3.8.1 and of Ch. 4's Jacobian
example, so the reader already knows this pose. Then $\cos q_2 = 0$ and
$M(q) = \begin{pmatrix} 2 & 1/3 \\ 1/3 & 1/3\end{pmatrix}$ exactly ($M_{11} = 1/6 + 1/12 + 2\cdot 0.25 +
1\cdot(1 + 0.25) = 2$, $M_{22} = 1/12 + 0.25 = 1/3$, $M_{12} = M_{22} + 0 = 1/3$); $\det M = 5/9 > 0$,
eigenvalues $0.2691$ and $2.0642$. Gravity: $g_1 = (1 + 1)(9.81)\cos\tfrac{\pi}{4} + 0.5\cdot 9.81
\cos\tfrac{3\pi}{4} = 1.5 \cdot 9.81 \cdot 0.70711 = 10.405$ N·m, $g_2 = 0.5\cdot 9.81\cdot(-0.70711) =
-3.468$ N·m. With $\dot q = (1, 0)$ rad/s: $K = \tfrac12 M_{11} = 1.000$ J, $h = m_2 L_1 r_2 \sin q_2 =
0.5$, and $C\dot q = (0,\; h\dot q_1^2) = (0, 0.5)$ N·m — joint 2 must push outward at half a
newton-meter just to keep the elbow angle fixed while the shoulder turns. Sweep $q_2 \in \{0, \pi/2,
\pi\}$ and $M_{11} \in \{3, 2, 1\}$: the stretched arm is three times harder to swing than the folded
one. Released from rest at this $q$ with $\tau = 0$, $E = V = 17.342$ J and must stay there.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"the arm is a different machine in every configuration."** Book colors: robot and
executed trajectory **orange**, planned path **purple**, obstacles **slate**; energy bars use the chrome
palette (kinetic/potential are not planning roles). Every widget autoplays and ships an SVG fallback.

- **Widget w17.1: Lagrangian Lab** — *flagship, wasm-sim.* Reach on a vertical Workbench, released from
  a configuration and swinging freely under gravity. **Manipulates:** headline — drag the two joints to
  set the release configuration (default $(\pi/4, \pi/2)$); behind a disclosure — $m_2$, $a_g$ (0 turns
  the Workbench horizontal), viscous damping, integrator (RK4 / semi-implicit Euler) and $h$; a
  **"Coriolis off"** toggle that zeroes $C\dot q$ in the integrated equations; seed re-roll draws a
  random release configuration (`SmallRng`, seed shown). **Observes:** the arm (orange) with its
  elbow-tip trace; three bars $K$, $V$, $E$; a phase portrait of $(q_1, \dot q_1)$; a torque gauge per
  joint showing $g(q)$ and $C\dot q$ as separate colored stacks. **Misconception killed:** "the
  velocity-product terms are a small correction you can drop" — with Coriolis off, $E$ drifts
  visibly within two swings and the arm gains energy from nowhere; and "$M\ddot q$ is the force" —
  the gauge shows $M\ddot q$ and $C\dot q$ trading places as coordinates change.
- **Widget w17.2: Inertia Ellipsoid** — *interactive animation.* Joint-velocity space $(\dot q_1,
  \dot q_2)$ with the unit-kinetic-energy ellipse $\{\dot q : \dot q^\top M(q)\dot q = 1\}$. **Manipulates:**
  headline — $q_2$ slider (the ellipse is independent of $q_1$: the text asks why, the reader answers
  "no $q_1$ in $M$"); disclosure — $m_2$, $r_2$, overlay of Ch. 4's Jacobian ellipse for the same $q$.
  **Observes:** semi-axes $1/\sqrt{\lambda_i}$ printed live; $M_{11}$ reading $3 \to 2 \to 1$ as $q_2$
  sweeps $0 \to \pi/2 \to \pi$; the principal directions rotating. **Misconception killed:** "inertia
  is a number" — it is a metric, and it is anisotropic.
- **Widget w17.3: Tumbling Body** — *interactive animation (added).* A uniform brick with principal
  moments $(I_{xx}, I_{yy}, I_{zz})$ spinning torque-free under Euler's body-frame equation.
  **Manipulates:** headline — initial spin axis chosen on a sphere (snap buttons: major / intermediate /
  minor axis, each perturbed by $1°$); disclosure — the three moments. **Observes:** the brick, the
  body-frame $\omega$ tracing its polhode on the kinetic-energy ellipsoid, $\|\mathcal{I}_s\omega_s\|$ and
  $K$ printed and constant. **Misconception killed:** "no torque means constant angular velocity" —
  about the intermediate axis the brick flips.

Dashboard layout: w17.1 full-width atop the C section, energy bars docked beneath; w17.2 half-width
beside the Christoffel derivation (hovering a symbol highlights the ellipse axis it bends); w17.3 a
half-width card beside Euler's equation.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`SMatrix<f64, N, N>` inertia matrices with const-generic DOF count,
Cholesky solves), `rapier2d` (tests only — the external referee), `rand` 0.9 (`SmallRng` for release
configurations), `eframe`/`egui` + `egui_plot` via `widget-kit` (Ch. 2), `plotters` (SVG fallbacks).
No dynamics library in the hot path: hand-rolling $M$, $C$, $g$ is the chapter.

**Module plan:**
- `crates/dynamics/` — **introduced here; imported by Chs. 18–21 and 23.**
  - `src/lib.rs` — `trait Dynamics<const N: usize>`, `forward_dynamics`, `State<N>`.
  - `src/lagrange.rs` — `christoffel` (closed form from a `d_mass` callback, or central differences),
    `coriolis_matrix`, energy helpers, the `skew_symmetry_defect` diagnostic.
  - `src/reach2r.rs` — the hand-derived 2R model (derivation 3); `src/reach3r.rs` — the Jacobian-
    assembled 3R model using `robots::reach::com_jacobians` from Ch. 2/4.
  - `src/constraint.rs` — `Pfaffian<N, K>`, `constrained_forward_dynamics`, projections `P`, `P_u`.
  - `src/rigid.rs` — `planar_inertia`, `parallel_axis`, `InertiaTensor`, `euler_body_step`.
  - `src/integrate.rs` — `rk4_step`, `semi_implicit_euler_step`, `simulate`.
  - `tests/rapier_crosscheck.rs`, `tests/energy_drift.rs`, `examples/two_r_numbers.rs`.
- `demos/ch17-demo/` — eframe app hosting w17.1–w17.3 (tabbed, lazy-iframe embedded).

**Key types & signatures** (compiles in spirit):

```rust
// crates/dynamics/src/lib.rs
use nalgebra::{SMatrix, SVector};

/// A fully actuated second-order mechanical system in generalized coordinates.
/// Everything downstream (time scaling, iLQR, kinodynamic RRT) talks to this trait.
pub trait Dynamics<const N: usize> {
    /// M(q): symmetric positive definite. We return the full matrix — the
    /// Cholesky factorization is cheap for N ≤ 3 and keeps the code readable.
    fn mass(&self, q: &SVector<f64, N>) -> SMatrix<f64, N, N>;
    /// The velocity-product vector C(q, q̇) q̇ = q̇ᵀ Γ(q) q̇ — returned as a vector, because
    /// the matrix C is not unique and only the product is a generalized force (C §10.2).
    fn velocity_product(&self, q: &SVector<f64, N>, qd: &SVector<f64, N>) -> SVector<f64, N>;
    /// g(q) = ∂V/∂q.
    fn gravity(&self, q: &SVector<f64, N>) -> SVector<f64, N>;
    /// Dissipation b(q, q̇); default: none.
    fn dissipation(&self, _q: &SVector<f64, N>, _qd: &SVector<f64, N>) -> SVector<f64, N> {
        SVector::zeros()
    }
    /// q̈ = M⁻¹(u − C q̇ − g − b). Default impl; override only for a faster recursive form.
    fn forward(&self, q: &SVector<f64, N>, qd: &SVector<f64, N>, u: &SVector<f64, N>)
        -> SVector<f64, N>
    {
        let rhs = u - self.velocity_product(q, qd) - self.gravity(q) - self.dissipation(q, qd);
        self.mass(q).cholesky().expect("M(q) must be positive definite").solve(&rhs)
    }
    fn potential_energy(&self, q: &SVector<f64, N>) -> f64; // K = ½ q̇ᵀ M q̇ is provided
}

// crates/dynamics/src/reach2r.rs
/// Choset Problem 10.2: planar 2R arm in a vertical plane. Units: kg, m, s.
pub struct Reach2R {
    pub l1: f64, pub r1: f64, pub m1: f64, pub i1: f64,
    pub l2: f64, pub r2: f64, pub m2: f64, pub i2: f64,
    pub a_g: f64, // 0.0 lays the Workbench flat
}
impl Reach2R {
    pub fn uniform(l1: f64, m1: f64, l2: f64, m2: f64, a_g: f64) -> Self; // r_i = L_i/2, I_i = m_i L_i²/12
    pub fn christoffel(&self, q: &SVector<f64, 2>) -> [SMatrix<f64, 2, 2>; 2]; // closed form, 4 nonzero
}

// crates/dynamics/src/constraint.rs
/// k Pfaffian constraints A(q) q̇ = 0 with analytic Ȧ(q, q̇); solves the (N+K)×(N+K)
/// KKT system of C eqs. 10.16–10.17 and returns (q̈, λ).
pub struct Pfaffian<const N: usize, const K: usize> { /* a(q), a_dot(q, q̇) closures */ }
pub fn constrained_forward_dynamics<const N: usize, const K: usize>(
    dyn_: &impl Dynamics<N>, c: &Pfaffian<N, K>,
    q: &SVector<f64, N>, qd: &SVector<f64, N>, u: &SVector<f64, N>,
) -> (SVector<f64, N>, SVector<f64, K>);

// crates/dynamics/src/integrate.rs
pub fn rk4_step<const N: usize>(
    dyn_: &impl Dynamics<N>, q: &SVector<f64, N>, qd: &SVector<f64, N>,
    u: &SVector<f64, N>, h: f64,
) -> (SVector<f64, N>, SVector<f64, N>);
```

**Worked end-to-end example:** `cargo run --example two_r_numbers -p dynamics` builds
`Reach2R::uniform(1.0, 2.0, 1.0, 1.0, 9.81)`, evaluates at $q = (\pi/4, \pi/2)$, and prints
`M = [[2.0000, 0.3333], [0.3333, 0.3333]]`, `g = [10.4051, -3.4684]`, `C q̇ = [0.0000, 0.5000]` for
$\dot q = (1, 0)$, `K = 1.0000 J`, `V = 17.3418 J`, then the $M_{11}$ sweep `3.0000 2.0000 1.0000`.
`#[test] fn reproduces_micro_example()` asserts all of these to $10^{-4}$. Three more tests pin the
chapter's claims: `christoffel_closed_form_matches_finite_differences` (2R closed form vs. central
differences with step $10^{-6}$, agreement to $10^{-7}$; the same finite-difference path is the *only*
one for 3R), `energy_drift_rk4` (zero torque from the micro-example's rest state, $h = 1$ ms, 10 s:
$|E(t) - E(0)|/E(0) < 10^{-6}$; the semi-implicit Euler run is asserted to be *worse* than $10^{-3}$,
so the comparison in w17.1 is honest), and `rapier_agrees` (Reach assembled in `rapier2d` as two
rigid bodies with revolute joints; one engine step at $\Delta t = 10^{-4}$ agrees with `forward` to
$10^{-3}$ relative, 2 s trajectories to $10^{-2}$ — the engine's own integrator sets that tolerance).

**Runnable artifact:** the WASM build of `dynamics` + `widget-kit` *is* w17.1; w17.2's ellipse is
`Reach2R::mass` eigen-decomposed per frame. The Integration lab drops the same `Reach2R` into the Ch. 2
Workbench with a PD controller holding a Ch. 12 waypoint under gravity; the torque trace it produces is
the limit the Ch. 18 time scaler inherits.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w17.1 | Lagrangian Lab | wasm-sim | dynamics + robots + widget-kit (eframe/egui_plot), SmallRng | drag release pose, mass/gravity/damping, Coriolis-off, integrator & h, seed re-roll | energy bookkeeping; velocity-product terms are not optional; $M\ddot q$ is not a force |
| w17.2 | Inertia Ellipsoid | interactive animation | dynamics + widget-kit | $q_2$ slider, $m_2$/$r_2$, Jacobian-ellipse overlay | $M(q)$ is a configuration-dependent metric |
| w17.3 | Tumbling Body | interactive animation | dynamics::rigid + manifold (Ch. 5 `SO3`) + widget-kit | spin-axis picker, principal moments | Euler's equation; intermediate-axis instability |
| — | Torque-gauge hook | animation (autoplay) | dynamics + sim | replay at 1×/2× | a path says nothing about feasibility in time |

## 7. Exercises & Extensions

1. **(F)** Derive the three-term Christoffel formula from $K = \tfrac12 \dot q^\top M(q)\dot q$; show
   that $C \to C + S$ with $S\dot q = 0$ leaves the motion unchanged but can break $\dot M - 2C$ skew.
2. **(F)** C Problem 10.1: a point mass in polar coordinates. Compute $M$, the symbols $\Gamma^1_{22} =
   -mr$, $\Gamma^2_{12} = \Gamma^2_{21} = mr$, and verify that unforced motions are Cartesian straight lines.
3. **(C, predict-then-verify)** In w17.2 the semi-axes at $q_2 = \pi/2$ are $1.928$ and $0.696$. Predict
   the two semi-axes at $q_2 = \pi$ (use $M_{11} = 1$, $M_{12} = 1/3 - 0.5 = -1/6$) before moving the
   slider; then explain in one sentence why the long axis rotates.
4. **(C)** In w17.1 switch Coriolis off and release from $(\pi/4, \pi/2)$. Does $E$ increase or
   decrease first, and why does the sign depend on the release configuration? Confirm with the
   torque-gauge stacks.
5. **(P)** Implement `Reach3R` by assembling $M$ from three center-of-mass Jacobians; add a property
   test that $M(q)$ is positive definite on 1000 seeded configurations and that
   `skew_symmetry_defect` is below $10^{-9}$.
6. **(P, stretch)** Implement Rusty's chassis as a planar rigid body with the no-side-slip Pfaffian
   constraint of Example 10.3.1 using `constrained_forward_dynamics`; verify $\lambda$ against C eq.
   10.28 and show the reduced two-equation form of C p. 361 emerges from `P`.

## 8. Modernization Notes

- **Kept, in full:** Choset's Lagrangian route (§10.1–10.2) rather than Newton–Euler — the inertia
  matrix as *the* object is exactly the view Ch. 18's path-constrained dynamics and Ch. 20's geometric
  controllability tests need; the Christoffel symbols with his geometric reading ("how unforced motions
  bend in these coordinates"); the constrained dynamics with projections (§10.3), because Ch. 20 reuses
  $A(q)$ and the knife-edge *is* Rusty; the rigid-body section (§10.4) including Euler's equation in
  both frames and the parallel-axis theorem.
- **Added beyond 2005:** the skew-symmetry property as a *test oracle* (classical, absent from
  Choset); an integrator section with an energy-drift contract (Choset assumes a Mathematica/Maple
  pipeline and never integrates); `rapier2d` as a second oracle — the book's "hand-rolled, externally
  refereed" pattern; the 3R model assembled from link Jacobians, the way modern rigid-body libraries
  build $M$ (Pinocchio, (verify) for a citation); w17.3 realizes C Problem 10.10.
- **Dropped/condensed:** the RP manipulator (C Examples 10.1.2, 10.2.1) survives only as a collapsible
  check — the cast is fixed and Reach is revolute; Euler-angle Lagrangian dynamics (end of §10.4.2) is
  a paragraph, since Ch. 5 already made the case for matrix coordinates; composite-body inertia (C eq.
  10.46) is a function, not a section.
- **Honesty items the prose must keep:** RK4 is not symplectic — the drift bound is empirical; `rapier2d`
  agreement is to engine tolerance, not machine precision; the 3R Christoffel symbols are
  finite-differenced, and the text says so where the 2R symbols are closed-form.
