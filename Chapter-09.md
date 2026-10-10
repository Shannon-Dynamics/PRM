# Chapter 9 — Roadmaps II: Sensor-Based Roadmaps and Silhouettes

> Part II — Classical Planners: Search, Potentials, Roadmaps, Cells · Estimated length: 10 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Chapter 8 ended on a cliff: in the plane, "equidistant to two obstacles" is a curve; in
$\mathbb{R}^3$ it is a sheet, and sheets are not roadmaps. This chapter climbs down two ways and
shows they are one idea. First, **intersect more equations** — equidistance to three obstacles in
$\mathbb{R}^3$ is a curve again (the generalized Voronoi graph) if the sheets cross
*transversally*; the curves need not connect, and patching them (the hierarchical GVG) is the real
work. Second, **sweep a slice** and track the extrema of a function on it — Canny's silhouette
curves — recursing where the trace breaks. Both obey Chapter 8's Morse vocabulary: critical points
are exactly where a roadmap can change connectivity. The "aha" is practical as well as topological:
**a robot can build a roadmap without a map**, because $d_i - d_j = 0$ is something a range sensor
measures, and a control law driving $\Gamma = \tfrac12 G^\top G$ to zero — a Lyapunov function —
lets Rusty *trace the GVG* with no discrete predictor–corrector. The price of exactness closes the
chapter: Canny's roadmap is complete and singly exponential in dimension, and the generalized
mover's problem is PSPACE-hard — which is why Part III exists. Reader path: Hook → C (play) →
F (rigor) → P (Rust) → Integration lab → Exercises.

**Story line**
1. **Hook** — Rusty drops into an unknown Apartment with only its range sensor; twenty seconds
   later a graph of the whole floor exists, drawn by the robot driving along it (autoplay of w9.1).
2. **Play (C)** — w9.1 GVG Explorer: meet points accumulate; pause and read the range readings
   defining the current edge. w9.2 Silhouette Sweep: a plane slides through a 3D C-space; extrema
   trace curves; the trace breaks; the plane *recurses*.
3. **Formalize (F)** — GVG via preimage theorem plus transversality; GVG disconnection and the
   HGVG patch; the Lyapunov box and tracing law $\dot q = \alpha v + \beta (DG)^\dagger G$; homing;
   rod-HGVG; Canny's roadmap, Lagrange lemmas, slice lemma; OPP; generalized gradient; complexity.
4. **Algorithm** — sensor-based GVG exploration (access, trace, detect nodes, branch, terminate);
   Lyapunov tracer; meet-point homing; Canny's recursive silhouette; OPP freeways and bridges.
5. **Implementation (P)** — `roadmap::gvg`: `GvgTracer` fed by range scans, incremental `GvgGraph`,
   a `canny` demonstrator on a synthetic implicit 3D C-space.
6. **Integration lab** — Rusty explores the Apartment by GVG; the graph is compared to Ch. 8's
   exact GVD; GVG path vs. shortest path over fifty seeded queries.
7. **Close** — exact roadmaps are complete and expensive; Part III trades completeness for speed.

## 2. Prerequisites & Position

- **Builds on:** Ch. 3 (range-sensor geometry $\rho$, $D$, $\nabla D$; the curve tracer — the
  Lyapunov law is its continuous cousin and §5.3.3 uses both), Ch. 5 (manifolds, tangent spaces,
  codimension), Ch. 6 (search on the GVG), Ch. 7 ($D$, nonsmoothness), Ch. 8 (GVD, preimage
  theorem, deformation retracts, Morse box — imported, not redefined).
- **Feeds into:** Ch. 10 (critical-point sensing via Lemma 5.5.2; boustrophedon critical points
  *are* Canny's), Ch. 11 (complexity motivates sampling), Ch. 23 (exploring the capstone's unknown
  Apartment), Appendix D (PSPACE-hardness), Appendix E (App. J.2 in full; only the box here).
- **Baseline sources:** C §5.3 intro, GVG in $\mathbb{R}^3$ [C refs. 105, 106], Figs. 5.15–5.16
  (b. 129–130); §5.3.1 transversality, $S_{ijk}$, $SS_{ijk}$, Eq. (5.3), Table 5.2 (b. 130–133);
  §5.3.2 connectivity, GVG cycles, second-order edges Eq. (5.4), periods, HGVG, projected gradient
  [C refs. 63, 106] (b. 133–135); §5.3.3 Lyapunov control Eq. (5.5), the $\dot\Gamma$ chain, homing
  [C refs. 108, 109, 202] (b. 136–138); §5.4 rod-HGVG, $RF_{ij}$, R-edges, junction regions
  [C ref. 107] (b. 138–141); §5.5 silhouettes (b. 141); §5.5.1 Canny's roadmap, bound
  $p^n (\log p)\, w^{O(n^4)}$ [C refs. 90–92, 189], Lemmas 5.5.1–5.5.2, slice lemma, Morse theory
  [C ref. 315] (b. 142–151); §5.5.2 OPP, freeways, bridge curves [C ref. 93], nonsmooth $\tilde D$,
  generalized gradient [C refs. 114, 104] (b. 151–155); App. G §G.2 (PSPACE) via Appendix D. Sister
  book: Ch. 24 (`…/chapters/ch24-exploration`, frontier exploration as the probabilistic cousin),
  Ch. 10 (`…/chapters/ch10-sensor-models`, what a range ring returns). Modernization set: none
  required; pointers marked (verify) in §8.

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped):

| Symbol | Meaning |
|---|---|
| $S_{ijk}$, $SS_{ijk}$, $F_{ijk}$ | triple-equidistant set; with distinct gradients; GVG edge in $\mathbb{R}^3$ |
| $\mathrm{codim}$; $T_x M_1 + T_x M_2 = T_x M$ | codimension (GVD has codim 1, GVG dim 1); transversal intersection at $x$ |
| $F_{kl}\big|_{F_{ij}}$, $F_{klp}\big|_{F_{ij}}$; $\pi_{T F_{ij}}$ | second-order GVG edge / meet point on sheet $F_{ij}$; projection onto its tangent space |
| $G(q)$, $DG$, $(DG)^\dagger$ | constraint vector (e.g. $[d_1 - d_2,\ d_1 - d_3]^\top$), its differential, pseudoinverse |
| $\alpha$, $\beta$, $v \in \mathrm{Null}(DG)$; $\Gamma = \tfrac12 G^\top G$ | tracing gains and tangent direction (C Eq. 5.5); Lyapunov function of the tracer |
| $RF_{ij}$, $RF_{ijk}$, $J_{ijk}$ | rod two-equidistant face, rod-GVG edge, junction region |
| $Q_\lambda$, $\pi_1, \pi_2, \pi_{12}$; $\Sigma(\cdot)$, $D(f,h)$ | slice at $\lambda$, coordinate projections; critical set, stacked differential of Lemma 5.5.2 |
| $\tilde d_i(p;\lambda)$, $\tilde D$ | distance restricted to a slice (C Eqs. 5.7–5.8) |
| $\partial D(q^\star)$, $\mathrm{Co}$, $Z(q^\star)$ | generalized gradient, convex hull, index set of closest obstacles |
| $p$, $w$ | number and maximum degree of the polynomials bounding the obstacles (Canny's bound) |

**Definitions** (display boxes):
- *Generalized Voronoi graph* (C §5.3.1): in $\mathbb{R}^m$, points equidistant to $m$ closest
  obstacles with distinct gradients; $\bigcup F_{i_1 \dots i_m}$, one-dimensional (C Table 5.2).
- *Transversality* (C §5.3.1): $M_1, M_2 \subset M$ meet transversally at $x$ if $T_x M_1 + T_x M_2
  = T_x M$; codimensions add; stable under perturbation; generalizes general position.
- *GVG cycle; second-order GVG edge; period; HGVG* (C §5.3.2): an edge homeomorphic to $S^1$; the
  set where $\mathcal{QO}_i, \mathcal{QO}_j$ tie closest and $\mathcal{QO}_k, \mathcal{QO}_l$ tie
  second-closest (C Eq. 5.4); a loop of second-order edges with a shared second-closest obstacle;
  the union GVG $\cup$ second-order GVG.
- **Lyapunov function (definition box — this chapter's; App. J.2 lives in Appendix E):** for
  $\dot q = X(q)$ with invariant set $A$ (here the GVG), a $C^1$ function $\Gamma \ge 0$ with
  $\Gamma = 0$ exactly on $A$ and $\dot\Gamma = \nabla\Gamma \cdot X < 0$ off $A$ in a neighborhood
  makes $A$ asymptotically stable: nearby trajectories converge to $A$. Choset: "an error function
  whose minimal value is zero" [C ref. 202].
- *Rod, rod-distance; R-edge; piecewise retract* (C §5.4): a planar segment, $\mathcal{Q} = SE(2)$,
  $d_i(q) = \min_{r \in R(q), c \in \mathcal{WO}_i} d(r,c)$ in the workspace; rod placements tangent
  to the planar point-GVG; deformation retracts of junction regions linked by R-edges.
- *Slice, critical slice, silhouette* (C §5.5, §5.5.1): $Q_\lambda = \pi_1^{-1}(\lambda)$; a slice
  through a critical point; the trace of extrema of $\pi_2|_{Q_\lambda}$ as $\lambda$ varies.
- *Freeway, bridge curve, channel, interesting critical point* (C §5.5.2): trace of slice-restricted
  local maxima of $\tilde D$; a curve joining freeways near an interesting critical point; a free
  slice component; where channels join or split.
- *Generalized gradient* (C §5.5.2 [C ref. 114]): $\partial D(q^\star) = \mathrm{Co}\{\nabla
  d_i(q^\star) : i \in Z(q^\star)\}$; $0 \in \partial D(q^\star)$ marks a local maximum of $D$.

**Derivations / theorems:**

1. **GVG edges in $\mathbb{R}^3$ are one-dimensional** (C §5.3.1). *Statement:* with $G(q) =
   [(d_i - d_j)(q), (d_i - d_k)(q)]^\top$, $SS_{ijk} = G^{-1}(0) \cap \{\text{distinct gradients}\}$
   is a 1-manifold. *Sketch (4 steps):* (i) each row of $DG$ is nonzero on $SS_{ij}$, $SS_{ik}$
   (Ch. 8); (ii) $DG$ loses rank only if $\nabla(d_i - d_j) = \alpha \nabla(d_i - d_k)$; (iii) that
   is a non-transversal intersection of the sheets' tangent spaces, removed by a generic
   perturbation — the transversality assumption; (iv) the preimage theorem gives $\dim = 3 - 2 = 1$.
   *Collapsible:* the three ways two lines meet (C Fig. 5.17), only the point stable; C Table 5.2
   (GVD codim 1, GVG dim 1 in every $\mathbb{R}^m$); $SS_{ijk} \ne SS_{ij} \cap SS_{ik}$ in general.
2. **The GVG is generically disconnected and the HGVG repairs it** (C §5.3.2). *Statement:* no
   one-dimensional deformation retract of a punctured 3-space exists [C ref. 63]; the 2-D GVD *is*
   connected, GVG edges lie on sheet boundaries ($F_{ijk} = \partial F_{ij} \cap \partial F_{ik}
   \cap \partial F_{jk}$), and second-order edges lead from a *period* to the missing cycle along
   $\dot c = -\pi_{T F_{ij}} \nabla d_k$. *Sketch (5 steps):* (i) the floor–ceiling sheet of
   C Fig. 5.19 has a hole bounded by the halo cycle; (ii) GVD connectedness confines the trouble to
   sheet boundaries; (iii) planar-GVG edges drawn *on* each sheet (second-order edges) are
   1-manifolds by the preimage theorem (C Eq. 5.4's gradient conditions); (iv) a period with common
   second-closest $\mathcal{QO}_k$ signals a cycle; (v) decreasing $d_k$ projected onto the sheet
   reaches $d_i = d_j = d_k$ unless the projected gradient vanishes (then no such edge exists).
   *Collapsible:* full HGVG connectivity is "tedious and challenging" and lives in [C ref. 106];
   the HGVG is *retract-like*, not a retract.
3. **The GVG tracing law is Lyapunov-stable** (C §5.3.3, Eq. 5.5). *Statement:* for $\dot q =
   \alpha v + \beta (DG)^\dagger G$, $v \in \mathrm{Null}(DG)$, $\|v\| = 1$, $(DG)^\dagger =
   DG^\top (DG\, DG^\top)^{-1}$: $\Gamma = \tfrac12 G^\top G$ has $\dot\Gamma = \beta\, G^\top G$,
   so $\beta < 0$ drives $q$ onto the GVG. *Sketch (5 steps):* (i) $\dot\Gamma = G^\top \dot G =
   G^\top DG\, \dot q$; (ii) the tangent term vanishes: $DG\, v = 0$; (iii) $DG\, DG^\top (DG\,
   DG^\top)^{-1} = I$; (iv) so $\dot\Gamma = \beta G^\top G \le 0$, strict off the GVG;
   (v) $DG\, DG^\top$ is invertible near the GVG [C ref. 108] — the neighborhood the definition box
   asks for. *Collapsible:* $\alpha$ as speed along the edge, $\beta$ as stiffness ("spring
   constants"); $\nabla G$ from a range ring by fitting a codimension-one plane through the $n$
   closest points (C §5.3.3, footnote 4 on the sign of $v$); planar $G = d_1 - d_2$ as a continuous
   Ch. 3 curve tracer.
4. **Meet-point homing is the same law with an empty null space** (C §5.3.3). *Statement:* with
   $G = [d_1 - d_2, d_1 - d_3, d_1 - d_4]^\top$ in $\mathbb{R}^3$ (two rows in the plane),
   $\mathrm{Null}(DG) = 0$ and $\dot q = \beta (DG)^\dagger G$ converges to the meet point, aiming
   at the center of the sphere (circle) through the closest points on the closest obstacles.
   *Sketch (3 steps):* (i) Derivation 3 with $v = 0$; (ii) $G = 0$ exactly at the meet point;
   (iii) the Newton-like step aims at the equidistant center. *Collapsible:* higher-order laws by
   swapping $G$ (second-order edges $[d_i - d_j, d_k - d_l]$).
5. **Silhouettes are critical sets; connectivity changes only at critical points** (C §5.5.1,
   Lemmas 5.5.1–5.5.2, slice lemma, Morse theory). *Statement:* $x$ is a critical point of $h|_{M}$,
   $M = f^{-1}(\cdot)$, iff $D(f,h)(x)$ (rows $Df$, then $Dh$) loses rank; the silhouette is
   $\Sigma(\pi_{12}|_S) = \bigcup_\lambda \Sigma(\pi_2|_{\pi_1^{-1}(\lambda)})$; between adjacent
   critical values the slice topology is constant [C ref. 315]. *Sketch (5 steps):* (i) Lagrange: at
   an extremum of $h$ on $f^{-1}(c)$, $\nabla h \parallel \nabla f$ (Lemma 5.5.1); (ii) for
   vector-valued $f, h$ this is rank loss of the stacked matrix (Lemma 5.5.2); (iii) the slice lemma
   exchanges "critical for $\pi_{12}$" with "critical for $\pi_2$ on each slice"; (iv) C's sphere:
   rows $[2q_1, 2q_2, 2q_3]$, $[1,0,0]$, $[0,1,0]$ lose rank iff $q_3 = 0$ — the equator; (v) a
   critical point of $\pi_1$ on the roadmap is where its tangent lies in the slice, i.e., where the
   number of silhouette pieces changes (C Figs. 5.30–5.33).
   *Collapsible:* the recursion — re-run on each critical slice with one fewer dimension down to
   dimension one, including the slices through $q_{start}, q_{goal}$ for access/departability;
   Canny's bound $p^n (\log p)\, w^{O(n^4)}$ [C refs. 91, 92]; the torus (C Figs. 5.31–5.32).
6. **OPP freeways and the generalized gradient** (C §5.5.2). *Statement:* slice-restricted local
   maxima of $\tilde D(p;\lambda) = \min_i \tilde d_i(p;\lambda)$ trace freeways, joined by bridge
   curves near interesting critical points; $\tilde D$ being nonsmooth at ties, maxima satisfy
   $0 \in \partial D(q^\star) = \mathrm{Co}\{\nabla d_i\}$, minima $\partial D = \{0\}$. *Sketch
   (4 steps):* (i) on a slice $\tilde D$ is smooth except at ties (C Fig. 5.37); (ii) at a tie the
   limiting gradients form the convex hull; (iii) a slice maximum means no direction increases all
   active $d_i$, i.e., $0 \in \mathrm{Co}$; (iv) bridges are needed where channels merge or split
   (surface normal $\parallel \nabla h$, C Fig. 5.35). *Collapsible:* C Problem 12 — OPP is a subset
   of the GVG for any slice direction; first-order data suffices.

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| Sensor-based GVG exploration (C §5.2.5, §5.3.3 steps 1–5) | `explore(scan_source) -> GvgGraph` | one edge traversal per GVG edge; DFS over meet points |
| GVG access (C §5.3.3) | `access(scan) -> Twist` | gradient ascent of $D$, then of $d$ on $SS_{ij}$ |
| Lyapunov tracer (C Eq. 5.5) | `trace_step(scan, alpha, beta) -> Twist` | $O(m^3)$ for the $m{-}1 \times m{-}1$ inverse |
| Meet-point homing (C §5.3.3) | `home(scan, beta) -> Twist` | Newton-like, local |
| HGVG period following (C §5.3.2) | `follow_period(scan) -> Option<GvgEdge>` | projected gradient descent of $d_k$ |
| Canny roadmap (C §5.5.1) | `canny(f, sweep_dir, depth) -> Roadmap` | $p^n (\log p)\, w^{O(n^4)}$ |
| OPP (C §5.5.2) | `opp(D, slice_dir, q_s, q_g) -> Option<Path>` | freeways + bridges at interesting critical points |

**Numeric micro-example** (the chapter's contract with the unit test). Two point obstacles
$c_1 = (0,0)$, $c_2 = (4,0)$, so the GVG (= GVD) is the line $x = 2$; Rusty at $q = (2.5, 1)$,
gains $\alpha = 1$, $\beta = -1$.
- $d_1 = \sqrt{7.25} = 2.6926$, $d_2 = \sqrt{3.25} = 1.8028$, $G = d_1 - d_2 = 0.8898$.
- $DG = \nabla d_1 - \nabla d_2 = (0.9285, 0.3714) - (-0.8321, 0.5547) = (1.7605, -0.1833)$;
  $(DG)^\dagger = DG^\top / (DG\, DG^\top) = (1.7605, -0.1833)/3.1333 = (0.5619, -0.0585)$.
- Tangent $v \perp DG$, oriented upward: $v = (0.1036, 0.9946)$.
- Correction $\beta (DG)^\dagger G = (-0.5000, 0.0521)$; velocity $\dot q = \alpha v + \text{corr}
  = (-0.3964, 1.0467)$ — leftward toward $x = 2$ while advancing along the edge.
- $\Gamma = \tfrac12 G^2 = 0.3959$, $\dot\Gamma = \beta G^2 = -0.7918 < 0$.
The correction's $x$-component is exactly $-0.5$, minus Rusty's offset from the edge: for two
point sites the Newton-like step $(DG)^\dagger G$ lands precisely on the bisector's $x$
(Exercise 1). The test pins all five numbers to $10^{-4}$.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, carried end to end: **"equidistance is something you can hear"** — three sonar
returns tie, and you are on the roadmap. Colors per the book code: obstacles slate, growing GVG and
meet points in the graph color, Rusty orange with faint rays, paths purple, start green, goal red.

- **Widget w9.1: GVG Explorer** — type: wasm-sim. Rusty in the Apartment, range ring only (Ch. 3
  sensor), no map; autoplays a seeded exploration: gradient-ascent access, Lyapunov tracing,
  meet-point homing, DFS over unexplored edges, termination. **Manipulates:** headline — the
  correction gain $\beta$ (slider $-0.1 \to -5$); disclosure: $\alpha$, sensor noise $\sigma$,
  start pose, seed re-roll. **Observes:** the two (then three) smallest rays highlighted; a $\Gamma$
  sparkline dipping whenever Rusty is nudged off the edge; meet points with unexplored-edge counts;
  the finished graph over Ch. 8's exact GVD. **Misconception killed:** "you need a map to build a
  roadmap" — the roadmap is a sensor quantity.
- **Widget w9.2: Silhouette Sweep** — type: interactive animation. A synthetic 3D C-space (C's
  bent-hole ellipsoid, Fig. 5.25, and a torus, Fig. 5.31) as a wireframe; a plane sweeps along
  $q_1$; extrema of $q_2$ on the slice are marked and traced. **Manipulates:** headline — the sweep
  parameter $\lambda$ (scrub); disclosure: sweep direction, critical slices, the recursion (the
  critical slice sweeps along $q_2$ with $q_3$-extrema). **Observes:** silhouette pieces appearing
  and vanishing *only* at critical points; the torus slice turning from one circle into two (C Fig.
  5.27-style before/after panels); recursive sub-silhouettes joining what the first pass left apart.
  **Misconception killed:** "the silhouette is the outline" — it is a projection's critical set.
- **Widget w9.3: Lyapunov Funnel** — type: interactive animation. The §3 two-site micro-world;
  place Rusty anywhere; the field $\alpha v + \beta (DG)^\dagger G$ is drawn and $\Gamma$ as a
  heightfield funnel. **Manipulates:** headline — $\beta$; disclosure: $\alpha$, drag the sites,
  homing with a third site. **Observes:** the trajectory sliding down the funnel onto the ridge
  $x = 2$; the correction's $x$-component equal to minus the offset for two point sites; with
  $\beta > 0$ it peels away. **Misconception killed:** "the controller needs the GVG's equation" —
  it needs only $G$ and $DG$, both read off the range ring.

Dashboard: w9.1 full-width atop the C section, $\Gamma$ sparkline beneath; w9.3 half-width beside
the Lyapunov box; w9.2 full-width in §5.5. All autoplay seeded defaults; static SVG fallback.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35 (`SMatrix` for $DG$, pseudoinverse), `petgraph` 0.8 (`GvgGraph`),
`parry2d` 0.30 (ground truth in tests only; the tracer sees scans), `rand` 0.9 (`SmallRng` noise,
seeded starts), `bugs` (Ch. 3 scan, `trace_curve`), `roadmap` (Ch. 8), `search` (Ch. 6).

**Module plan:**
- `crates/roadmap/src/gvg/` — **added to the Ch. 8 crate by this chapter.** `mod.rs` (`GvgGraph`,
  `MeetNode`, `Edge`), `scan.rs` (closest-$k$ obstacles as local minima of $\rho$ with gradients —
  C §4.3.1/§5.3.3), `tracer.rs` (`GvgTracer`: `trace_step`, `home_step`, `access_step`),
  `explore.rs` (five-step exploration, DFS bookkeeping), `hgvg.rs` (second-order edges, period
  following), `canny.rs` (silhouette of an implicit $f(q) = 0$ in $\mathbb{R}^3$ by slice sweeping
  and Newton polishing, recursion to depth 2), `opp.rs` (slice maxima of $D$, generalized-gradient
  test), `examples/{lyapunov_step, explore_apartment, ellipsoid_silhouette}.rs`.
- `web/lib/roadmap/gvg/` — TypeScript port for w9.1 and w9.3; w9.2's silhouettes are precomputed by
  `canny.rs` and shipped as data (3D sweeping is not ported).

**Key types & signatures:**

```rust
use nalgebra::{SMatrix, SVector, Vector2};

/// The k nearest hits in a range ring: distance and unit direction away from each (C §4.3.1).
pub struct Nearest<const K: usize> { pub d: [f64; K], pub grad: [Vector2<f64>; K], pub obstacle_id: [Option<usize>; K] }
pub fn nearest_from_scan<const K: usize>(scan: &bugs::RangeScan) -> Nearest<K>;

/// C Eq. (5.5) in the plane: G = d_1 - d_2 (edge tracing) or [d_1-d_2, d_1-d_3] (homing).
pub struct GvgTracer { pub alpha: f64, pub beta: f64 }
pub struct Twist { pub v: Vector2<f64> }   // body-frame velocity for Rusty's controller (Ch. 2)
impl GvgTracer {
    /// Edge tracing: q̇ = α v + β (DG)† G, v ∈ Null(DG) along `heading`; also returns Γ = ½ GᵀG.
    pub fn trace_step(&self, n: &Nearest<2>, heading: Vector2<f64>) -> (Twist, f64);
    /// Meet-point homing: Null(DG) is empty, so q̇ = β (DG)† G aims at the equidistant center.
    pub fn home_step(&self, n: &Nearest<3>) -> (Twist, f64);
    /// Access: gradient ascent of D until two readings tie (Ch. 8 Lemma 5.2.1), then trace.
    pub fn access_step(&self, n: &Nearest<2>) -> Twist;
}
/// DG† = DGᵀ (DG DGᵀ)⁻¹; the inverse exists near the GVG (C §5.3.3), else `None`.
pub fn pseudo_inverse<const R: usize, const C: usize>(dg: &SMatrix<f64, R, C>) -> Option<SMatrix<f64, C, R>>;
/// Incrementally built topological map (C §5.2.5 exploration, §5.3.3 steps 1–5).
pub struct GvgGraph { pub graph: petgraph::graph::UnGraph<MeetNode, Edge>, pub frontier: Vec<(NodeIndex, usize)> }
pub struct MeetNode { pub q: Vector2<f64>, pub clearance: f64, pub departures: Vec<Vector2<f64>>, pub explored: Vec<bool> }
pub struct Edge { pub polyline: Vec<Vector2<f64>>, pub length: f64 }
pub enum Event { OnEdge, MeetPoint(MeetNode), BoundaryPoint, Done }
pub struct Explorer<'r> { pub robot: &'r mut sim::Rusty, pub tracer: GvgTracer, pub graph: GvgGraph }
impl Explorer<'_> {
    /// One tick: scan; access/trace/home by state; flag a meet point on "a sudden change in one of
    /// the two closest obstacles"; DFS to the next unexplored edge.
    pub fn tick(&mut self, dt: f64) -> Event;
}
// src/gvg/canny.rs — demonstrator on implicit surfaces f(q) = 0 in R^3
pub struct Silhouette { pub curves: Vec<Vec<SVector<f64, 3>>>, pub critical_points: Vec<SVector<f64, 3>>, pub depth: usize }
/// Sweep π_1 slices; π_2-extrema by Newton on rank loss of D(f, π_12) (Lemma 5.5.2); recurse once
/// on critical slices where the piece count changes.
pub fn canny(f: &dyn Fn(SVector<f64, 3>) -> f64, grad_f: &dyn Fn(SVector<f64, 3>) -> SVector<f64, 3>, n_slices: usize) -> Silhouette;
```

**Worked end-to-end example:** `cargo run --example lyapunov_step -p roadmap` places the two sites
and Rusty at $(2.5, 1)$ and prints `G = 0.8898  DG = (1.7605, -0.1833)  DG+ = (0.5619, -0.0585)
v = (0.1036, 0.9946)  qdot = (-0.3964, 1.0467)  Gamma = 0.3959  dGamma = -0.7918`; `#[test] fn
lyapunov_step_matches_text()` pins each to $10^{-4}$ and integrates the law for 200 steps of
$dt = 0.01$, asserting $\Gamma$ monotone non-increasing and $|x - 2| < 10^{-3}$ at the end.
`cargo run --example explore_apartment --seed 9` runs the full exploration and prints meet-point
count, edge count, and path driven; `#[test] fn explored_graph_matches_exact_gvd()` asserts every
meet point lies within $2\sigma + 5$ cm of a Ch. 8 exact meet point and edge counts agree. `cargo
run --example ellipsoid_silhouette` sweeps C's bent-hole ellipsoid, printing the four critical
slices (C Fig. 5.26) and piece counts before/after each; the sphere unit test asserts the silhouette
is the equator $q_3 = 0$ to $10^{-6}$ (Derivation 5 step iv).

**Runnable artifact:** w9.1 and w9.3 are `web/lib/roadmap/gvg/` ports of `GvgTracer`, `Explorer`,
and `nearest_from_scan` over the ported Ch. 3 scan and Ch. 2 Rusty; w9.2 renders silhouettes
precomputed by `canny`, shipped as JSON, `__checks__.ts` verifying the sphere case. The Integration
lab compares GVG-path length with the Ch. 6 shortest path on fifty seeded Apartment queries.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w9.1 | GVG Explorer | wasm-sim | `roadmap::gvg` → `web/lib/roadmap/gvg/`, `bugs` scan, `sim` Rusty, seeded `SmallRng` noise | $\beta$ slider, $\alpha$, noise, start, seed re-roll | a roadmap is a sensor quantity; exploration = graph DFS over meet points |
| w9.2 | Silhouette Sweep | interactive animation | `roadmap::gvg::canny` (precomputed data) + Canvas 3D wireframe | $\lambda$ scrub, sweep direction, critical slices, recursion toggle | silhouettes are critical sets; connectivity changes only at critical points |
| w9.3 | Lyapunov Funnel | interactive animation | `roadmap::gvg::tracer` → TS port | $\beta$, $\alpha$, drag sites, homing mode | $\dot\Gamma = \beta G^\top G$; the controller needs only $G$, $DG$ |
| — | Unknown-Apartment hook | animation (autoplay) | `roadmap::gvg`, `sim` | none (replay of w9.1) | the whole floor's graph from a range ring |
| f9.4 | Transversality trio | static-svg | build-time SVG | hover | parallel / point / overlap: only the point is stable (C Fig. 5.17) |

## 7. Exercises & Extensions

1. **(F)** Prove that for two point sites the $x$-component of $(DG)^\dagger G$ equals the signed
   distance to the bisector exactly (the micro-example's $0.5$); test numerically for a point and a
   segment site, and explain what you find.
2. **(F)** Prove that for any slice direction the OPP roadmap is a subset of the GVG (C Problem 12)
   via the generalized-gradient characterization of slice maxima.
3. **(C, predict-then-verify)** In w9.3 set $\beta = -3$, Rusty at $(3, 0)$. From $\dot\Gamma =
   \beta G^2$, predict the steps of $dt = 0.05$ until $|x - 2| < 0.01$ before running; then explain
   the discrepancy the $\alpha v$ term introduces by moving the robot along the edge.
4. **(C)** In w9.1 raise the sensor noise until exploration fails to terminate; decide from the
   meet-point list whether a meet point was spurious or missed, and relate it to "a sudden change in
   one of the two closest obstacles."
5. **(P)** Implement `follow_period` in `hgvg.rs` for a box-in-a-room world extruded to 3D
   (C Fig. 5.19); show the inner halo cycle is reached from the period around it.
6. **(P, stretch)** Implement the rod-GVG of C §5.4 in the Workbench: $d_i(q)$ as segment–obstacle
   distance (`parry2d`), $RF_{ijk}$ by the tracer with $G \in \mathbb{R}^2$ over $SE(2)$, R-edges as
   rod placements tangent to the Ch. 8 planar GVD; render C Fig. 5.22's panels.

## 8. Modernization Notes

- **Kept from Choset, deliberately:** the full transversality argument (where genericity is made
  precise); the HGVG *problem* — no one-dimensional retract of punctured 3-space — as the honest
  reason sensor-based roadmaps are hard beyond the plane; the Lyapunov derivation line by line, the
  book's first stability proof and the pattern Ch. 21's steering controllers repeat; Canny's roadmap
  with both Lagrange lemmas and the slice lemma (Ch. 10's sensing is Lemma 5.5.2); OPP and the
  generalized gradient.
- **Added beyond 2005:** the Lyapunov definition box placed here (App. J.2 moved to Appendix E);
  GVG exploration identified with DFS over a graph the robot is still building, cross-linked to
  frontier exploration in the sister book's Ch. 24; the complexity closing — Canny's bound beside
  PSPACE-hardness (Appendix D) — as the stated motivation for Part III; pointers, marked (verify):
  Voronoi-based exploration and topological mapping in modern SLAM; GVD/GVG from ESDFs (sister
  Ch. 19).
- **Dropped/condensed:** the rod-HGVG (C §5.4) to a definition, a remark, and a stretch exercise;
  HGVG connectivity to [C ref. 106]; the ISS figure (C Fig. 5.16) to a sentence; Canny recursion
  beyond depth 2 (the demonstrator recurses once).
- **Honesty items the prose must keep:** (i) the tracing law is *locally* stable — $DG\, DG^\top$
  must be invertible, which fails exactly at meet points, where homing takes over; (ii) HGVG
  connectivity is cited, not proved; (iii) meet-point detection from noisy range data is heuristic
  ("sudden change"), and w9.1 shows it failing; (iv) Canny's algorithm is complete but has never
  been a practical planner; the chapter says so and says why Part III follows.
