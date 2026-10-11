# Chapter 20 — Nonholonomic Systems I: Controllability

> Part V — Dynamics, Trajectories, and Constraints · Estimated length: 11 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Hitch cannot slide sideways, yet every reader knows Hitch can end up one meter to the left of where
it started, parallel-parked, by wiggling. This chapter turns that folk knowledge into a theorem.
The "aha": **a velocity constraint is not a configuration constraint, and the Lie bracket is the
exact bookkeeping of what wiggling buys.** Following *drive* for $\epsilon$, *steer* for $\epsilon$,
then undoing both in order does not return Hitch home; it leaves a residue of size $\epsilon^2$ in a
direction neither input produces directly — the bracket $[g_1, g_2]$. Stack brackets until they span
the tangent space and Chow's theorem says the system reaches everything nearby (small-time locally
controllable); stop short and Frobenius says it is trapped on a lower-dimensional leaf forever. The
second half asks the same of second-order systems — Reach with a dead third motor, Rusty with mass —
where drift complicates the bookkeeping and the symmetric product of Lewis and Murray halves the
work. Everything here is *analysis*; Chapter 21 cashes it in as steering algorithms.

Story line:
1. **Hook** — autoplay: Hitch is told "0.5 m to the left, same heading." A holonomic ghost slides
   there in one second; Hitch needs 14 forward–back cycles, and a counter shows the net sideways
   displacement growing *quadratically* in the wiggle amplitude.
2. **Play (C)** — w20.1: drive the four-flow loop by hand, shrink $\epsilon$, watch the residue
   shrink as $\epsilon^2$ while its direction converges to the bracket arrow.
3. **Formalize (F)** — tangent spaces, vector fields, flows; distributions as spans and as
   annihilated sets; the bracket from a Taylor expansion of the loop; filtrations, Frobenius (w20.3).
4. **Controllability** — control-affine systems with drift; the four notions and their ladder
   (w20.4); Chow/LARC; Sussmann's good and bad brackets; WPPS drift (w20.2).
5. **Second-order systems (F, advanced)** — kinetic-energy metric, covariant derivative, symmetric
   product; Lewis–Murray tests at rest; nonholonomic constraints via the projection $P$.
6. **Implementation (P)** — the `nonholonomic` crate: `VectorField`, `lie_bracket`, filtration
   rank tests; the unit test pins the unicycle bracket and the car's rank ladder $(2, 3, 4)$.
7. **Integration lab** — the tests run on Hitch with and without trailer, then on Reach's 3R with
   the third joint unpowered; close on the question Ch. 21 answers: *controllable — with what controls?*

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (Hitch's kinematic model in `robots`, the Lot), Ch. 4 (Reach's Jacobian;
  $SE(2)$ slices for the lab), Ch. 5 (`trait Manifold`, $T_q\mathcal{Q}$, charts — this chapter computes
  in local coordinates on purpose), Ch. 17 (`dynamics`: $M(q)$, $C(q,\dot q)$, Christoffel symbols,
  Pfaffian constraints — imported, never re-derived).
- **Feeds into:** Ch. 21 (every steering method presumes the STLC verdicts proven here; kinematic
  reductions reuse the symmetric product), Ch. 23 (the trailer jackknife is a loss of regularity of
  the filtration at $\theta_1 = \pm\pi/2$), Appendix B (glossary card).
- **Baseline sources:** C §12.1 (12.1.1–12.1.3, b. 402–414), §12.2 (b. 414–416), §12.3
  (12.3.1–12.3.2, b. 416–424: Theorems 12.3.1, 12.3.3, 12.3.5), §12.4 preamble and §12.4.1
  (b. 424–434: Theorems 12.4.1–12.4.2, Examples 12.4.3–12.4.4), §12.4.3 (b. 438–440, Example
  12.4.9); Problems 1–10, 13–16, 19–22, 24, 26–29 as exercise stock. §12.4.2 is deliberately *not*
  here (Ch. 21). Sister book: Ch. 3 (`…/chapters/ch03-geometry-of-motion`, Lie groups), Ch. 4
  (`…/chapters/ch04-rusty-and-sensors`, the no-slip constraint as kinematics). Modernization set:
  Solà, Deray & Atchuthan micro-Lie theory (verify) for the bracket-to-commutator bridge.

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped; everything else per TOC.md §2):

| Symbol | Meaning |
|---|---|
| $M$, $x \in M$, $n = \dim M$ | state manifold and state ($M = \mathcal{Q}$ kinematic; $M = T\mathcal{Q}$, $x = (q, \dot q)$ second-order) |
| $T_xM$, $TM$, $T^*_xM$, $\omega(x)$, $\phi^g_t(x)$ | tangent space, tangent bundle, cotangent space, covector (one-form), flow of $g$ for time $t$ |
| $\mathcal{D} = \mathrm{span}(\mathcal{G})$, $\mathcal{D}_k$, $\overline{\mathcal{D}}$, $\mathrm{Lie}(\mathcal{G})$ | distribution; $k$-th filtration step; involutive closure; Lie algebra generated |
| $g_0$, $g_i$, $u \in U \subset \mathbb{R}^m$ | drift field, control fields, control vector and set |
| $\mathcal{U}^\pm$, $\mathcal{U}^+$; $R^V(x, \le T)$ | control-set classes (origin interior to the convex hull / merely spanning); states reachable within time $T$ without leaving neighborhood $V$ |
| $\langle v_1, v_2\rangle = v_1^\top M(q) v_2$, $\nabla_{Y_1}Y_2$, $\langle Y_1 : Y_2\rangle$ | kinetic-energy metric, covariant derivative, symmetric product |
| $Y_i(q)$, $\mathrm{Sym}(\mathcal{Y})$ | columns of $M^{-1}(q)T(q)$; symmetric closure |
| $P = I - M^{-1}A^\top(AM^{-1}A^\top)^{-1}A$, $\tilde Y_i = PY_i$, $\tilde\nabla$ | constraint projection, constrained inputs and connection |
| Hitch: $q = (x, y, \phi, \theta)$, $L$, $\gamma$, $\theta_1$, $d_1$ | rear-axle position, steering angle, heading (Choset's ordering), wheelbase, steering limit; trailer angle relative to the car, hitch length |

**Definitions** (boxed one-liners):
- *Vector field, flow, integral curve*: $g: M \to TM$ with $g(x) \in T_xM$; the solution $\phi^g_t(x)$
  of $\dot x = g(x)$; the set $\{\phi^g_t(x)\}_{t \in \mathbb{R}}$.
- *Distribution*: positive form $\mathrm{span}\{g_i(x)\}$; negative form $\{\dot x : \omega_j(x)\dot x
  = 0\}$ — for the unicycle $\omega = [-\sin\theta, \cos\theta, 0]$; *regular* if its dimension is constant.
- *Nonholonomic constraint*: a velocity constraint $f(q, \dot q) = 0$ that is not the derivative of
  any configuration constraint; underactuation yields acceleration constraints that may or may not
  integrate down.
- *Lie bracket; filtration; involutive closure*: $[g_1, g_2] = \frac{\partial g_2}{\partial x}g_1 -
  \frac{\partial g_1}{\partial x}g_2$; $\mathcal{G}_{i+1} = \mathcal{G}_i \cup \{[g_j, g_k] : g_j \in
  \mathcal{G}_1, g_k \in \mathcal{G}_i\}$, $\mathcal{D}_i = \mathrm{span}(\mathcal{G}_i)$;
  $\overline{\mathcal{D}} = \mathcal{D}_\infty$, involutive iff $\mathcal{D} = \overline{\mathcal{D}}$,
  nilpotent of order $k$ if all Lie products of degree $> k$ vanish.
- *Integral manifold, foliation, leaf*: the set reachable from $x_0$ along fields in $\mathcal{D}$;
  when smaller than $M$, the distinct integral manifolds are the leaves of a foliation.
- *Control-affine system*: $\dot x = g_0(x) + \sum_i g_i(x)u_i$; *driftless* if $g_0 = 0$;
  *symmetric* if driftless with $U \in \mathcal{U}^\pm$. *Controllable / accessible / STLA / STLC
  from $x$*: every goal reachable in finite time / $R^M(x, \le T)$ has interior / $R^V(x, \le T)$ has
  interior for all $V, T$ / $R^V(x, \le T)$ contains a neighborhood of $x$ for all $V, T$.
- *Good and bad brackets* (Sussmann): bad if $g_0$ appears an odd number of times and every $g_i$
  an even number; good otherwise. *WPPS drift*: every point is nonwandering.
- *Symmetric product; STLCA / STLCC / STLEC*: $\langle Y_1 : Y_2\rangle = \nabla_{Y_1}Y_2 +
  \nabla_{Y_2}Y_1$, bad if every $Y_i$ appears an even number of times; configuration-accessible /
  configuration-controllable / reachable zero-velocity configurations fill a neighborhood of $q$.

**Derivations / theorems** (statement → numbered sketch → collapsible contents):

1. **The four-flow loop produces the bracket at order $\epsilon^2$** (C eq. 12.3–12.5, Problem 29).
   *Statement:* $\phi^{-g_2}_\epsilon \circ \phi^{-g_1}_\epsilon \circ \phi^{g_2}_\epsilon \circ
   \phi^{g_1}_\epsilon(x_0) = x_0 + \epsilon^2[g_1, g_2](x_0) + O(\epsilon^3)$, no $O(\epsilon)$ term.
   *Sketch (5):* (i) one flow: $x + \epsilon g + \tfrac{\epsilon^2}{2}\frac{\partial g}{\partial x}g +
   O(\epsilon^3)$; (ii) compose $g_2$, re-expanding at the moved point: cross term $\epsilon^2
   \frac{\partial g_2}{\partial x}g_1$; (iii) compose $-g_1$: the $\epsilon g_1$ and self-terms cancel,
   cross term $-\epsilon^2\frac{\partial g_1}{\partial x}g_2$ appears; (iv) compose $-g_2$: the rest
   cancels; (v) the survivor is the bracket. *Collapsible:* the full expansion;
   the unicycle loop in closed form, $\epsilon[\cos\theta - \cos(\theta + \epsilon),\ \sin\theta -
   \sin(\theta + \epsilon),\ 0]$, so the $O(\epsilon^3)$ remainder is a number w20.1 plots.
2. **Frobenius: integral manifolds exist iff the distribution is involutive** (C §12.1.3).
   *Statement:* a regular $m$-dimensional $\mathcal{D}$ integrates to $m$-dimensional leaves through
   every point iff involutive; a regular filtration terminates at finite $k$ with $\mathcal{D}_k =
   \overline{\mathcal{D}}$. *Sketch (4):* (i) necessity — flows stay in a leaf, so the four-flow loop
   does, so brackets are tangent to it; (ii) sufficiency — straighten the fields by their flows;
   (iii) regularity makes $\dim\mathcal{D}_i$ strictly increase or stop, bounded by $n$; (iv) a system
   confined to a $k < n$ leaf satisfies no controllability property. *Collapsible:* C Example 12.1.7
   ($g_1 = [x_1\cos x_3, x_2\sin x_3, 0]^\top$,
   $g_2 = \partial_3$, $\det = x_1x_2$, nine leaves — w20.3's data); the Philip Hall basis [C ref. 380].
3. **Chow's theorem / LARC** (C Thm. 12.3.1 [C ref. 112]) **with Sussmann's refinement** (C Thm.
   12.3.3 [C ref. 401]). *Statement:* STLA from $x$ if (and for analytic fields only if)
   $\mathrm{Lie}\{g_0, \dots, g_m\}(x) = T_xM$, any $U \in \mathcal{U}^+$; symmetric systems are then
   STLC. With drift: STLC at $x$ if $g_0(x) = 0$, $U \in \mathcal{U}^\pm$, good brackets up to degree
   $k$ satisfy the LARC, and every bad bracket of degree $j \le k$ is a combination of good brackets
   of degree $< j$. *Sketch (6):* (i) each Lie product direction is realized by nested four-flow
   loops, net motion $O(\epsilon^{\deg})$ in time $O(\epsilon)$; (ii) full rank ⇒ an open set of
   directions ⇒ STLA; (iii) symmetry reverses every loop, so each direction comes with both signs
   ⇒ STLC; (iv) a bad bracket's loop has the $u_i$ with even exponents — one sign only, drift-like —
   and must be neutralized by lower-degree good brackets; (v) the degree-1 bad bracket is $g_0$,
   hence equilibria only; (vi) connected + STLC everywhere ⇒ controllable.
   *Collapsible:* the implication ladder (C Fig. 12.12); Choset's four counterexamples (one-way watch
   knob; $g_0 = [x_2^2, 0]^\top$; double integrator; forward-only unicycle); why the linearizations
   are not controllable (contrast Appendix E).
4. **Global controllability from recurrent drift** (C Thm. 12.3.5 [C ref. 289]). *Statement:*
   WPPS drift, $U \in \mathcal{U}^\pm$, LARC ⇒ controllable on $M$. *Sketch (3):* LARC gives forward
   local reachability; recurrence supplies the backward motions symmetry would have given; chain
   neighborhoods along any path. *Collapsible:* $g_0 = \tfrac12[-x_2, x_1]^\top$, $g_1 = e_1$, $\det =
   -\tfrac12$; energy conservation on compact $\mathcal{Q}$ ⇒ WPPS by Poincaré recurrence [C ref. 26].
5. **The symmetric product halves the computation for simple mechanical systems** (C §12.4.1
   [C refs. 285, 286]). *Statement:* for $\nabla_{\dot q}\dot q = \sum Y_i u_i$, written on
   $T\mathcal{Q}$ with $g_0 = [\dot q;\ -M^{-1}C\dot q]$, $g_i = [0;\ Y_i]$: $[g_i, g_j] = 0$,
   $[g_0, g_i] = [-Y_i;\ 0]$ at rest, $[g_i, [g_0, g_j]] = [0;\ \langle Y_i : Y_j\rangle]$. From
   $(q, 0)$: STLA iff $\mathrm{Sym}(\mathcal{Y})(q) = T_q\mathcal{Q}$, STLC if also every bad
   symmetric product is a combination of lower-degree good ones (Thm. 12.4.1); STLCA iff
   $\mathrm{Lie}(\mathrm{Sym}(\mathcal{Y}))(q) = T_q\mathcal{Q}$, STLCC and STLEC under the same
   neutralization, equilibrium controllable if everywhere (Thm. 12.4.2). *Sketch (5):* (i) intrinsic
   acceleration $\nabla_{\dot q}\dot q = \ddot q + M^{-1}C\dot q \in T_q\mathcal{Q}$ (the centripetal
   term on $S^1$ is the picture); (ii) $\nabla_{Y_1}Y_2 = \frac{\partial Y_2}{\partial q}Y_1 +
   M^{-1}Y_1^\top\Gamma Y_2$ with Ch. 17's Christoffel symbols; (iii) block-differentiate for the three
   identities; (iv) $\mathrm{Sym}$ = reachable velocities from rest, $\mathrm{Lie}(\mathrm{Sym})$ =
   reachable configurations; (v) carry Sussmann's bookkeeping over. *Collapsible:* Reach's 3R with
   $u_3 = 0$ — its free third link is
   Choset's planar body with thrusters (Ex. 12.4.3): $Y_1 = [\cos q_3, \sin q_3, 0]^\top$, $Y_2 =
   [-\sin q_3, \cos q_3, -d]^\top$, $\langle Y_1 : Y_2\rangle = [d\sin q_3, -d\cos q_3, 0]^\top$, rank 3,
   bad $\langle Y_2 : Y_2\rangle = dY_1$ neutralized ⇒ STLC at rest; the hopper (Ex. 12.4.4) as STLEC
   but not STLA.
6. **Nonholonomic constraints enter through the projection $P$** (C §12.4.3, Example 12.4.9).
   *Statement:* for $M\ddot q + \dot q^\top\Gamma\dot q = Tu + A^\top\lambda$, $A\dot q = 0$:
   $\tilde\nabla_{\dot q}\dot q = \sum\tilde Y_i u_i$ with $\tilde Y_i = PY_i$, $\tilde\nabla_{\tilde Y_1}
   \tilde Y_2 = P\nabla_{\tilde Y_1}\tilde Y_2$; Theorems 12.4.1–12.4.2 apply, but velocities live in
   the $(n_\mathcal{Q} - k)$-dimensional $\mathcal{D}$, so the best case is STLEC with
   $\mathrm{Sym}(\tilde{\mathcal{Y}}) = \mathcal{D}$. *Sketch (4):* Ch. 17's multiplier elimination
   gives $P$; project inputs; project the connection; rerun the tests. *Collapsible:* the knife-edge
   (Rusty with mass $m$, inertia $I$, forward force, torque):
   $A = [\sin\theta, -\cos\theta, 0]$, $\tilde Y_1 = [\cos^2\theta, \sin\theta\cos\theta, 0]^\top/m$,
   $\tilde Y_2 = e_3/I$, $\langle\tilde Y_1 : \tilde Y_2\rangle = -\tfrac{\tan\theta}{I}\tilde Y_1$ so
   $\mathrm{Sym} = \mathrm{span}$ (secretly kinematic — Ch. 21's "maximally reducible"); the
   degree-2 determinant $\cos^2\theta/(m^2I^2)$ vanishes at $\theta = \pm\pi/2$, rescued at degree 3
   by $2/(m^2I^4)$ ⇒ STLCA and STLEC everywhere.

**Named algorithms** (Choset numbers none of these; names match the Rust):

| Algorithm | Signature | Complexity |
|---|---|---|
| `lie_bracket` | `(g1, g2) -> impl VectorField`, $\partial g_2\,g_1 - \partial g_1\,g_2$ at $x$ | $O(n^2)$ per evaluation with analytic Jacobians; $O(n^2)$ field evaluations by central differences |
| `filtration` (Chow's test) | `(fields, x, max_deg, tol) -> Filtration { dims, basis }` | Hall basis has $\frac1k\sum_{e \mid k}\mu(e)m^{k/e}$ products at degree $k$; rank by SVD |
| `larc` / `sussmann_stlc` | `(sys, x, max_deg) -> Verdict` | one filtration + symmetry check / good-bad tags + least-squares neutralization |
| `symmetric_product` / `lewis_murray` | `(Y1, Y2, &dynamics::Model)` / `(Ys, model, q, max_deg) -> MechVerdict` | each product $O(n_\mathcal{Q}^3)$ from Christoffel symbols; Sym closure then Lie closure |
| `constrained_inputs` | `(Ys, A, model, q) -> Vec<impl VectorField>` ($\tilde Y_i = PY_i$) | $O(k^3 + n_\mathcal{Q}^2k)$ for $P$ |

**Numeric micro-example** (the contract with the unit test in §5). *Unicycle at $x_3 = \pi/6$:*
$g_1 = [\cos x_3, \sin x_3, 0]^\top = [0.8660, 0.5, 0]^\top$, $g_2 = e_3$; $\partial g_2/\partial x =
0$ and $\partial g_1/\partial x$ has the single nonzero column $[-\sin x_3, \cos x_3, 0]^\top$, so
$[g_1, g_2] = -(\partial g_1/\partial x)g_2 = [\sin x_3, -\cos x_3, 0]^\top = [0.5, -0.8660, 0]^\top$ —
sideways, the forbidden direction. The exact four-flow loop with $\epsilon = 0.1$ moves
$[0.005424, -0.008396, 0]$ against the prediction $\epsilon^2[g_1, g_2] = [0.005, -0.008660, 0]$:
agreement to $O(\epsilon^3) \approx 4 \times 10^{-4}$. $\det[g_1\ g_2\ [g_1, g_2]] = 1$ for all $x_3$.
*Hitch's car (C eq. 12.34), $q = (x, y, \phi, \theta)$:* $g_1 = [\cos\theta, \sin\theta, 0,
\tfrac1L\tan\phi]^\top$, $g_2 = e_3$; $g_3 = [g_1, g_2] = [0, 0, 0, -\tfrac1L\sec^2\phi]^\top$;
$g_4 = [g_1, g_3] = \tfrac{\sec^2\phi}{L}[-\sin\theta, \cos\theta, 0, 0]^\top$; $\det[g_1\ g_2\ g_3\ g_4]
= -\sec^4\phi/L^2 \ne 0$ for $|\phi| < \pi/2$. With $L = 1$ at $q = 0$ the columns are $e_1, e_3, -e_4,
e_2$ and the determinant is $-1$. Dimension ladder $(2, 3, 4)$: $\mathcal{D}_3 = \overline{\mathcal{D}}$;
the car is STLC everywhere whatever the steering limit $\gamma$ — which is why a car that can only
turn left reaches every pose a normal car can (C §12.5.6).

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"you cannot go there, but you can wiggle there — and the wiggle has a
direction."** Colors per TOC.md: Hitch and its executed trajectory **robot (orange)**, the ghost's
target **goal (red)**, start **start (green)**, bracket arrows and reachable sets **graph (blue)**,
pillars **obstacle (slate)**. Every widget autoplays a seeded default and degrades to a build-time SVG.

- **Widget w20.1: Lie Bracket Parallel-Parker** — wasm-sim (flagship). Hitch in an empty bay.
  **Manipulates:** headline slider $\epsilon$ ($0.02$–$0.5$); disclosure: field pair ($g_1, g_2$
  of the unicycle reduction, or $g_1, g_3$ of the 4-state car producing the sideways $g_4$), loop
  count, exact-vs-$\epsilon^2$ overlay, trailer on/off (a fifth state), seed re-roll. **Observes:**
  the four legs in orange; the blue residue arrow; a log–log inset of $\|\text{residue}\|$ vs.
  $\epsilon$ with slope 2; the residue direction converging on the bracket arrow; the $O(\epsilon^3)$
  gap from the micro-example. **Misconception killed:** "sideways is not in a car's reachable set"
  and "the net motion of drive–steer–undrive–unsteer is zero."
- **Widget w20.2: Reachable Set Grower** — wasm-sim. $R^V(x, \le T)$ grown by dense sampling of
  piecewise-constant controls. **Manipulates:** headline toggle of available inputs (drive only /
  steer only / both / both plus a bracket macro); disclosure: control-set class ($\mathcal{U}^\pm$
  vs. forward-only $\mathcal{U}^+$), $T$, radius of $V$, seed. **Observes:** the blue set in the
  $(x, y)$ projection and an $(x, \theta)$ slice — a curve (a leaf), a patch that does *not* contain
  a neighborhood (STLA, honestly), the full blob (STLC); forward-only keeps the set open but pushes
  $x$ to its boundary (controllable, not STLC — the lemma Dubins needs in Ch. 21).
  **Misconception killed:** "accessible, STLA, STLC and controllable are four words for one thing."
- **Widget w20.3: Foliation Leaves** — interactive animation of C Example 12.1.7. **Manipulates:**
  drag the start point in $\mathbb{R}^3$. **Observes:** random control sequences shade the leaf the
  system stays on — nine leaves: a line, four half-planes, four quadrants — with $\det = x_1x_2$
  live, dropping as the start crosses a coordinate plane. **Misconception killed:** "a rank test at
  one configuration settles controllability everywhere" (the trailer's jackknife is the same thing).
- **Widget w20.4: Controllability Ladder** — static-svg with hover. C Fig. 12.12 with its conditional
  arrows annotated; each node reveals its counterexample and the Lot or Workbench system exhibiting
  it. **Misconception killed:** "linearize and apply the Kalman rank test."

Dashboard layout: w20.1 full width atop the C section with the $\epsilon$ slider docked beneath;
w20.2 and w20.3 half-width beside the control-set and foliation subsections; w20.4 inline in §12.3's
prose. Derivation 1's four-term equation sits in a sticky panel lit term by term as w20.1 draws each leg.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`SVector`, `SMatrix`, SVD rank with tolerance), `rand_pcg` 0.9 (`Pcg64`;
only the reachable-set sampler and the hook target are stochastic), `robots` (Ch. 2 Hitch, the Lot),
`dynamics` (Ch. 17 `Model`, `Pfaffian`), `manifold` (Ch. 5 `SE2`, `S1` wrap), `collide` (Ch. 2, lab
only). Brackets, filtrations and symmetric products are hand-rolled, per the book's policy.

**Module plan** — `crates/nonholonomic/` (introduced here; Ch. 21 adds `steer/`, `lattice/`,
`reduction/`, `car_grid.rs`): `src/field.rs` (`trait VectorField<const N>`, Jacobians,
`Bracket<F, G>`); `src/filtration.rs` (Hall-basis generation, `filtration`, `larc`, `sussmann_stlc`,
`Verdict`); `src/mechanical.rs` (`symmetric_product`, `sym_closure`, `lewis_murray`,
`constrained_inputs`); `src/systems.rs` (`Unicycle`, `Car`, `CarTrailer`, `KnifeEdge`,
`Reach3RUnderactuated`, analytic Jacobians); examples `chow_car`, `reach_dead_joint`; tests
`bracket_laws.rs`, `micro_example.rs`.

**Key types & signatures** (compiles in spirit):

```rust
use nalgebra::{SMatrix, SVector};

/// A smooth vector field on a chart of an N-dimensional state manifold. Brackets need Jacobians,
/// so the trait carries one; the default central difference keeps nested brackets usable, and
/// concrete systems override it analytically.
pub trait VectorField<const N: usize> {
    fn eval(&self, x: &SVector<f64, N>) -> SVector<f64, N>;
    fn jacobian(&self, x: &SVector<f64, N>) -> SMatrix<f64, N, N> {
        central_difference(|y| self.eval(y), x, 1e-6)
    }
}

/// [g1, g2] = (∂g2/∂x) g1 − (∂g1/∂x) g2  — Choset eq. 12.5, same sign convention.
pub struct Bracket<F, G>(pub F, pub G);
impl<const N: usize, F: VectorField<N>, G: VectorField<N>> VectorField<N> for Bracket<F, G> {
    fn eval(&self, x: &SVector<f64, N>) -> SVector<f64, N> {
        self.1.jacobian(x) * self.0.eval(x) - self.0.jacobian(x) * self.1.eval(x)
    }
}

/// The dimension ladder dim D_1 ≤ dim D_2 ≤ … at one point: dims = [2, 3, 4] for the car.
pub struct Filtration<const N: usize> { pub dims: Vec<usize>, pub basis: Vec<SVector<f64, N>>, pub terminated: bool }
pub fn filtration<const N: usize>(fields: &[&dyn VectorField<N>], x: &SVector<f64, N>,
                                  max_deg: usize, tol: f64) -> Filtration<N>;

pub enum Verdict { Stla, StlcSymmetric, StlcSussmann, Undecided, ConfinedToLeaf { dim: usize } }
pub struct ControlAffine<const N: usize> { /* drift: Option<_> (None ⇒ driftless), controls, control_set: U± | U+ */ }
/// Chow / LARC with the symmetry shortcut (Thm. 12.3.1); Sussmann's test at rest (Thm. 12.3.3).
pub fn larc<const N: usize>(sys: &ControlAffine<N>, x: &SVector<f64, N>, max_deg: usize) -> Verdict;
pub fn sussmann_stlc<const N: usize>(sys: &ControlAffine<N>, x: &SVector<f64, N>, max_deg: usize) -> Verdict;

/// ⟨Y1 : Y2⟩ = ∇_{Y1} Y2 + ∇_{Y2} Y1 using Ch. 17's Christoffel symbols; Lewis–Murray verdicts.
pub fn symmetric_product<const Q: usize>(y1: &dyn VectorField<Q>, y2: &dyn VectorField<Q>,
                                         model: &dynamics::Model<Q>) -> impl VectorField<Q>;
pub enum MechVerdict { Stla, Stlc, Stlca, Stlec, EquilibriumControllable, Undecided }
pub fn lewis_murray<const Q: usize>(inputs: &[&dyn VectorField<Q>], model: &dynamics::Model<Q>,
                                    q: &SVector<f64, Q>, max_deg: usize) -> MechVerdict;
/// Ỹ_i = P Y_i, P = I − M⁻¹Aᵀ(AM⁻¹Aᵀ)⁻¹A (C §12.4.3); A is Ch. 17's Pfaffian type.
pub fn constrained_inputs<const Q: usize, const K: usize>(inputs: &[&dyn VectorField<Q>],
    constraint: &dynamics::Pfaffian<Q, K>, model: &dynamics::Model<Q>) -> Vec<Box<dyn VectorField<Q>>>;
```

**Worked end-to-end example:** `cargo run --example chow_car -p nonholonomic` prints, for the
unicycle at $x_3 = \pi/6$, the bracket `[0.500000, -0.866025, 0.000000]`, the exact loop residue for
$\epsilon = 0.1$ `[0.005424, -0.008396, 0.000000]` beside the prediction `[0.005000, -0.008660,
0.000000]`, and `det = 1.000000`; then for Hitch with $L = 1$ at $q = 0$: `dims = [2, 3, 4]`,
`det = -1.000000`, `Verdict::StlcSymmetric`; with the trailer `dims = [2, 3, 4, 5]`, stalling at
$\theta_1 = \pi/2$. `#[test] fn reproduces_micro_example()` asserts every number to $10^{-6}$.
`tests/bracket_laws.rs` property-tests seeded random polynomial fields: antisymmetry, the Jacobi
identity $[f, [g, h]] + [h, [f, g]] + [g, [h, f]] = 0$ at 200 points, and the linear case
$[Ax, Bx] = (BA - AB)x$ (Choset Problem 8 — note the sign). `examples/reach_dead_joint.rs` runs
`lewis_murray` on Reach's 3R with $u_3 = 0$ and prints `Stlc` away from $q_2 \in \{0, \pi\}$.

**Runnable artifact:** the WASM build of `nonholonomic` *is* w20.1 and w20.2 — the loop the reader
drives calls `Bracket::eval` and `filtration`. The Integration lab drops Hitch with the trailer into
the Lot with Ch. 2's collision checker, sweeps $\theta_1$ toward $\pm\pi/2$, and shows the rank
ladder losing a rung exactly where Ch. 23's jackknife failure will appear.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w20.1 | Lie Bracket Parallel-Parker | wasm-sim | nonholonomic + robots + manifold, Pcg64 | $\epsilon$ slider, field pair, loop count, overlay, trailer toggle, seed | bracket = $\epsilon^2$ residue of the four-flow loop; sideways is reachable |
| w20.2 | Reachable Set Grower | wasm-sim | nonholonomic + robots, Pcg64 | input toggle, control-set class, $T$, $V$, seed | accessible vs. STLA vs. STLC vs. controllable, as sets |
| w20.3 | Foliation Leaves | interactive animation | nonholonomic (Example 12.1.7 fields) | drag start point | integral manifolds; non-regular distributions; rank is pointwise |
| w20.4 | Controllability Ladder | static-svg (hover) | build-time SVG + CSS hover | hover nodes | the implication diagram and its conditional arrows |
| — | Hitch vs. holonomic ghost hook | animation (autoplay) | robots + nonholonomic | none (seeded replay) | why wiggling works and why it is slow |

## 7. Exercises & Extensions

1. **(F)** Reach's 2R on the horizontal Workbench with $u_2 = 0$ (Choset Problem 19): write $g_0, g_1$
   on $T(T^2)$, argue WPPS from energy conservation on compact $T^2$, attempt the LARC for Theorem
   12.3.5, and explain why the arm is *not* controllable with $u_1 = 0$.
2. **(F)** Prove the car (12.34) is STLC (Problem 13) by exhibiting the ladder $(2, 3, 4)$
   symbolically; add the trailer, $\dot\theta_1 = \tfrac{u_1}{d_1}\sin\theta_1 - \tfrac{u_1}{L}\tan\phi$,
   show one more degree is needed, and locate where the filtration is not regular.
3. **(C, predict-then-verify)** In w20.1 with $\epsilon = 0.2$, $x_3 = \pi/6$, predict the residue
   from $\epsilon^2[g_1, g_2]$ and the sign of the $O(\epsilon^3)$ error from Derivation 1's closed
   form; verify in the widget. Repeat for the car's $[g_1, g_3]$ pair and explain why the sideways
   residue scales as $\epsilon^3$ in the *original* inputs.
4. **(C)** In w20.2, switch Hitch to forward-only and find the smallest $T$ for which the reachable
   set contains a point *behind* $x$; relate it to $\rho$ and to w20.4's "controllable, not STLC" rung.
5. **(P)** Implement `impl VectorField<6>` for the free third link of Reach (Problems 14–16) with
   the analytic Jacobian, reproduce $\det[g_1 \cdots g_6] = d^4$ in a test, and show `sussmann_stlc`
   returns `StlcSussmann` at rest only when $a_g = 0$.
6. **(P, stretch)** Replace the Choset recursion in `filtration` by a Philip Hall basis, benchmark
   bracket evaluations for the car-with-trailer to degree 5, then add a small symbolic backend so the
   car's $\det = -\sec^4\phi/L^2$ is produced as an expression.

## 8. Modernization Notes

- **Kept, essentially whole:** C §12.1–12.3 is the cleanest short route from vector fields to
  Chow's theorem in the robotics literature; the four-flow derivation, filtration, Frobenius, the
  four controllability notions, Sussmann's brackets and the Lewis–Murray tests are reproduced in
  full. The unicycle survives as Rusty's kinematic skeleton; the planar body with thrusters as the
  free third link of Reach, exactly as Choset uses it in §12.5.7; the hopper and knife-edge stay.
- **Added beyond 2005:** (i) the $\epsilon^2$ residue made *measurable* (w20.1's log–log inset),
  the $O(\epsilon^3)$ remainder shown, not asserted; (ii) reachable sets grown numerically so the
  four notions become four pictures; (iii) a one-page bridge from vector-field brackets to matrix
  commutators and the sister book's Lie-group chapter (Solà, Deray & Atchuthan (verify));
  (iv) property-tested bracket laws as the correctness contract (Choset's Problem 9 asks for symbolic
  code; we ship tested numeric code); (v) a typed `Verdict` so Ch. 21's planners can refuse systems
  that are merely accessible.
- **Dropped or condensed:** §12.4.2 moves whole to Ch. 21 (a planning tool used only by §12.5.7);
  §12.1's general-manifold caveats become one honest paragraph plus a pointer to Ch. 5.
- **Honesty items the prose must keep:** STLC for second-order systems is a statement *at rest*; the
  LARC is necessary only for analytic fields; Sussmann's condition is sufficient, not necessary (the
  single-thruster body is STLA and controllable yet fails it); nothing here produces a plan — Ch. 21 does.
