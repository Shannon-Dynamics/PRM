# Chapter 3 — Bug Algorithms: Planning with a Contact Sensor

> Part I — Foundations: Robots, Worlds, and Configuration Space · Estimated length: 9 web pages · Difficulty: Foundational

## 1. Purpose & Story Arc

Choset opens his book with the Bug algorithms because they prove something surprising with almost
no mathematics: a point robot that knows only where the goal is, how far it has gone, and whether it
is touching something can be *guaranteed* to reach any reachable goal, with a path whose length is
bounded in advance. This chapter keeps that order of business and sharpens it. The "aha": **a planner
is two behaviors and a switching rule, and the whole difference between a heuristic and a theorem is
the switching rule.** Bug1 switches exhaustively (circumnavigate, then leave from the best point) and
pays for its certainty; Bug2 switches greedily (leave the first time the fixed m-line is closer) and
can be made arbitrarily worse than Bug1 by a comb-shaped obstacle; Tangent Bug buys shorter paths
with a range sensor by *seeing* the discontinuities of $\rho_R$ and heading for them. Underneath all
three sits a problem the rest of the book keeps meeting: *how do you follow a curve you have not been
given?* Choset's answer — treat the offset curve as the zero set of $G(x) = D(x) - W^*$ and trace it
with a predictor–corrector justified by the implicit function theorem — becomes this chapter's owned
artifact `trace_curve`, which Chs. 7–10 import to follow level sets, Voronoi edges, the GVG, and
critical-point slices.

Story line:

1. **Hook** — w3.1 autoplays a three-lane race: Bug1, Bug2, and Tangent Bug on the same map, same
   query. Bug2 wins. Then the map flips to Choset's figure 2.4 and Bug2 loses badly.
2. **Play (C)** — drag the goal and watch hit points, leave points, and length counters race the
   printed bounds; open w3.3 Tangent Scope to see what a range sensor "sees."
3. **Formalize (F)** — Bug1 (C Alg. 1) and Bug2 (C Alg. 2) stated precisely; the bounds (2.1) and
   (2.2) proved; the comb construction; Tangent Bug (C Alg. 3) with $\rho$, $\rho_R$, $d_{reach}$,
   $d_{followed}$.
4. **Algorithm** — boundary following as curve tracing: the tangent is $(\nabla D)^\perp$, the offset
   curve is $G^{-1}(0)$, and the predictor–corrector (eq. 2.5) with its two theorems (App. D).
5. **Implementation (P)** — the `bugs` crate over the Ch. 2 `RangeSensor` and `Collision`;
   `trace_curve(G, DG)` as a generic, reusable tracer; property tests for both bounds.
6. **Integration lab** — Rusty, shrunk to a point by Ch. 2's disc reduction, runs all three Bugs in
   the Apartment from the range sensor alone; the Bug1 bound is checked on every seeded query.
7. **Close** — what the Bugs assume (perfect position, bounded world, a point) and what breaks
   without it; the chapters that reuse the tracer.

## 2. Prerequisites & Position

- **Builds on:** Ch. 1 (Bug1 informally; the 28-step text world; the completeness taxonomy), Ch. 2
  (`Scene` + `Collision::distance` as the world; `RangeSensor::scan` as $\rho_R$; `World::apartment`
  via `Scene::from_world`; the disc-to-point reduction, Derivation 3 of Ch. 2).
- **Feeds into:** Ch. 7 §4.3.1 (distance and gradient from range data are the sensor-based potential;
  `trace_curve` follows level sets), Ch. 8 §5.2.5 (the GVD traced with the same tracer), Ch. 9 §5.3.3
  (the sensor-based GVG is Bug boundary-following grown up), Ch. 10 §6.2.4 (sensor-based coverage
  follows slices with it), Ch. 6 and Appendix D (the completeness notion Bug1 has), Ch. 23 (Rusty's
  unknown-Apartment navigation starts from the Bug behaviors before replanning takes over).
- **Baseline sources:** C §2.1 Bug1 and Bug2 (b. 17–22; Algs. 1–2; eqs. 2.1–2.2; figures 2.1–2.4),
  §2.2 Tangent Bug (b. 23–30; eq. 2.3; Alg. 3; figures 2.5–2.13), §2.3.1 the tangent line (b. 31–32),
  §2.3.2 distance and gradient (b. 32–35; eq. 2.4 — the sonar-cone and centerline-model detail is
  cross-linked, not re-taught), §2.3.3 continuation methods (b. 35–37; eq. 2.5), App. D.1 the implicit
  function theorem and D.2 the Newton–Raphson convergence theorem (b. 487–488). Bug1/Bug2 are
  [C ref. 301]; Tangent Bug [C refs. 216, 217]; curve tracing [C ref. 232]. Sister cross-links: Ch. 4
  (`https://shannon-dynamics.github.io/ProbabilisticRoboticRust/chapters/ch04-rusty-and-sensors`)
  and Ch. 10 (`…/chapters/ch10-sensor-models`) for sonar beam patterns, the centerline model, and
  what real range sensors return. Modernization: comparative Bug-algorithm studies (Ng & Bräunl 2007,
  verify) for the "which Bug wins where" question w3.1 lets the reader answer empirically.

## 3. Foundation (F) — Mathematical Core

Chapter notation table:

| Symbol | Meaning |
|---|---|
| $q^H_i$, $q^L_i$ | $i$-th hit point and leave point |
| m-line | Bug1: the segment from $q^L_{i-1}$ to $q_{goal}$; Bug2: the fixed segment $q_{start}q_{goal}$ |
| $p_i$, $n_i$ | perimeter of $\mathcal{WO}_i$; number of times the Bug2 m-line crosses $\partial\mathcal{WO}_i$ |
| $L_{Bug1}$, $L_{Bug2}$ | path lengths |
| $B_r(x)$ | open ball of radius $r$ about $x$; the workspace is bounded: $\mathcal{W} \subset B_r(x)$ for some finite $r$ |
| $O_i$, $T$ | endpoints of the intervals of continuity of $\rho_R(x, \cdot)$; the point of the sensing circle on $x\,q_{goal}$ |
| $d_{reach}$, $d_{followed}$ | Tangent Bug's two bookkeeping distances |
| $W^*$ | the safety offset of the followed curve |
| $G(x) = D(x) - W^*$, $DG$, $D_Y G$ | the offset-curve defining function; its differential; the differential on the correcting hyperplane $Y$ |
| $\Delta\lambda$ | the predictor step along the tangent |

**Definitions** (definition boxes):

- **Contact (tactile) sensor.** Reports whether the point robot touches $\partial\mathcal{WO}_i$;
  equivalently a range sensor with $R = 0$.
- **Raw distance function** (C eq. 2.3). $\rho(x, \theta) = \min\{\lambda \ge 0 : x + \lambda(\cos\theta,
  \sin\theta) \in \bigcup_i \mathcal{WO}_i\}$; the saturated $\rho_R$ equals $\rho$ when $\rho < R$ and
  $\infty$ otherwise.
- **Interval of continuity.** A connected set of boundary points $x + \rho(x,\theta)(\cos\theta,
  \sin\theta)$ over which $\rho_R(x, \cdot)$ is finite and continuous; its endpoints $O_i$ arise where
  one obstacle occludes another (the ray is tangent) or where the boundary leaves sensing range.
- **Distance to the nearest obstacle** (C eq. 2.4). $D(x) = \min_{c \in \bigcup_i \mathcal{WO}_i} d(x, c)
  = \min_s \rho(x, s)$; $\nabla D(x)$ is the unit vector along the minimizing ray, away from the obstacle.
- **Offset curve.** $\{x : D(x) = W^*\}$; its tangent at $x$ is $(\nabla D(x))^\perp$.
- **Followed vs. blocking obstacle.** The obstacle whose boundary Tangent Bug tracks vs. the nearest
  in-range obstacle intersecting the segment $x\,q_{goal}$; equal when boundary-following begins.

**Derivations / theorems:**

1. **Bug1 is complete and $L_{Bug1} \le d(q_{start}, q_{goal}) + 1.5\sum_i p_i$** (C eq. 2.1).
   *Statement:* in a bounded workspace with finitely many obstacles, Bug1 reaches $q_{goal}$ when a
   path exists, otherwise halts with "unreachable" in finite time, and its path obeys the bound.
   *Sketch (6 steps):* (i) each leave point $q^L_i$ is the perimeter point of $\mathcal{WO}_i$ closest
   to the goal, so $d(q^L_i, q_{goal}) < d(q^H_i, q_{goal})$ unless the goal is unreachable; (ii) hence
   no obstacle is hit twice and at most $n$ circumnavigations occur; (iii) each costs at most $p_i$ plus
   at most $p_i/2$ to return to the leave point the shorter way; (iv) motion-to-goal segments move
   straight toward the goal from points successively closer to it, so they total at most
   $d(q_{start}, q_{goal})$; (v) sum; (vi) failure test: if stepping from $q^L_i$ toward the goal
   re-enters $\mathcal{WO}_i$, the goal is enclosed. *Collapsible:* why "grazing" needs no boundary
   following, and why the sum ranges over *all* obstacles (an upper bound, not an estimate).
2. **Bug2's bound and why it can lose** (C eq. 2.2). *Statement:* $L_{Bug2} \le d(q_{start}, q_{goal})
   + \tfrac12 \sum_i n_i p_i$, and for every $K$ there is a workspace with $L_{Bug2} > K \cdot L_{Bug1}$.
   *Sketch (5 steps):* (i) Bug2 may leave $\mathcal{WO}_i$ only at a point of the fixed m-line, so at
   most $n_i$ candidates; (ii) half lie on the "wrong side" (moving toward the goal would re-enter);
   (iii) each leave may cost nearly a full perimeter; (iv) sum with the straight-line budget $d$;
   (v) the comb: a boundary crossing the m-line $n_1$ times forces Bug2 to walk $\approx n_1 p_1/2$
   while Bug1 walks $\le 1.5 p_1$ — C figure 2.4. *Collapsible:* the exhaustive-vs-greedy reading
   (C b. 22) — the book's first instance of the greedy trade-off that returns with best-first search in Ch. 6.
3. **$D(x) = \min_s \rho(x, s)$ and $\nabla D$ is the minimizing ray** (C problem 2.1). *Statement:*
   the nearest obstacle point lies on a ray from $x$ at distance $\rho(x, s^*)$ for the minimizing
   bearing $s^*$, and $\nabla D(x) = -(\cos s^*, \sin s^*)$ wherever the minimizer is unique. *Sketch
   (3 steps):* every obstacle point is on some ray; the closest point along ray $s$ is at $\rho(x,s)$;
   minimize over $s$ and apply the envelope argument for a point and a closed set. *Collapsible:*
   non-uniqueness on the medial axis — exactly the set Ch. 8's GVD is made of; $D$ is continuous
   everywhere, differentiable off that set.
4. **The offset curve is a smooth curve where $DG$ is surjective** (C §2.3.3 + App. D.1). *Statement:*
   with coordinates $(\lambda, y)$ along the tangent and normal, if $D_Y G$ is invertible at a zero of
   $G$, the zero set is locally the graph of a unique smooth $y(\lambda)$. *Sketch (4 steps):* $D$ is
   smooth off the medial axis with $|\nabla D| = 1$; in the $(\lambda, y)$ frame $DG = [\,0 \;\; D_Y G\,]$
   with $D_Y G = \pm 1 \ne 0$; apply Theorem D.1.1 with $n = 1$; the graph *is* the offset curve locally.
   *Collapsible:* the general $G : \mathbb{R}^m \times \mathbb{R}^n \to \mathbb{R}^n$ used by Chs. 8–9
   (equidistance to several obstacles), and that nonsingular matrices form an open set, so full rank
   persists in a neighborhood.
5. **The corrector converges quadratically** (App. D.2, Theorem D.2.1). *Statement:* if $f(y^*) = 0$,
   $\|(Df(y^*))^{-1}\| \le \beta$, and $Df$ is $\gamma$-Lipschitz on $B_\rho(y^*)$ with $\gamma \le
   2/(\rho\beta)$, then $y_{h+1} = y_h - (Df(y_h))^{-1} f(y_h)$ from any $y_0 \in B_\rho(y^*)$ stays in
   the ball and $\|y_{h+1} - y^*\| \le a\|y_h - y^*\|^2$ with $a = \beta\gamma/(2(1 - \rho\beta\gamma))$.
   *Sketch (3 steps):* Taylor-expand about $y_h$; bound the remainder by $\gamma\|y_h - y^*\|^2/2$;
   multiply by the bounded inverse. *Collapsible:* the specialization in the tracer — $f = G$ on the
   correcting line, $Df = D_Y G$ a scalar computable from $\nabla D$, i.e., from sensor data (eq. 2.5).
6. **Length of a convex obstacle's offset curve** (for the tracer's test). *Statement:* for a convex
   polygon of perimeter $p$, the offset curve at distance $W^*$ has length $p + 2\pi W^*$. *Sketch
   (3 steps):* translated edges contribute $p$; one arc of radius $W^*$ per vertex; the turning angles
   sum to $2\pi$. *Collapsible:* the Steiner formula; reflex vertices lose their arcs; large $W^*$
   makes the curve self-intersect.

**Named algorithms:**

| Name | Signature | Complexity |
|---|---|---|
| `BUG1` (C Alg. 1) | `(sensor: Contact, q_start, q_goal) -> Result<Path, Unreachable>` | $L \le d + 1.5\sum p_i$; $O(1)$ sensing per step |
| `BUG2` (C Alg. 2) | `(sensor: Contact, q_start, q_goal) -> Result<Path, Unreachable>` | $L \le d + \tfrac12\sum n_i p_i$ |
| `TANGENT BUG` (C Alg. 3) | `(sensor: RangeSensor(R), q_start, q_goal) -> Result<Path, Unreachable>` | $O(n_{rays})$ per step for $\{O_i\}$, $d_{reach}$, $d_{followed}$ |
| `continuity_intervals` | `(scan: &[f64], R) -> Vec<Interval>` | $O(n_{rays})$ |
| `distance_from_scan` / `gradient_from_scan` (C eq. 2.4) | `(scan) -> f64` / `(scan) -> Vector2` | $O(n_{rays})$ |
| `trace_curve` (App. D predictor–corrector, C eq. 2.5) | `(G, DG, x0, step, stop) -> Vec<Point2>` | $O(\text{steps} \times \text{Newton iters})$; quadratic corrector |

**Numeric micro-example** (the contract with the tests in §5). Continuous plane, point robot,
$q_{start} = (0,0)$, $q_{goal} = (10, 0)$, two axis-aligned obstacles: $\mathcal{WO}_1 = [2,4] \times
[-1,1]$ ($p_1 = 8$) and $\mathcal{WO}_2 = [6,7] \times [-2,2]$ ($p_2 = 10$). **Bug1:** motion-to-goal
along the $x$-axis hits $q^H_1 = (2,0)$ after 2; circumnavigates 8; the boundary point nearest the goal
is $(4,0)$ at distance 6, reached the short way in 4 (both ways are 4 — the obstacle is symmetric about
the m-line, so this is exactly the worst case $p_1/2$); motion-to-goal hits $q^H_2 = (6,0)$ after 2;
circumnavigates 10; $q^L_2 = (7,0)$ reached in 5 ($= p_2/2$ again); motion-to-goal 3.
$L_{Bug1} = 2 + 8 + 4 + 2 + 10 + 5 + 3 = \mathbf{34}$. Bound: $10 + 1.5(8 + 10) = \mathbf{37} \ge 34$;
the slack of 3 is exactly the obstacles' widths along the m-line, which the straight-line budget never
pays. **Bug2:** fixed m-line $y = 0$; hit $(2,0)$ after 2; turn left: $(2,0) \to (2,1) \to (4,1) \to
(4,0)$ is 4; $(4,0)$ is on the m-line, closer (6 < 8), and the step toward the goal is free, so leave;
2 to $(6,0)$; $(6,0) \to (6,2) \to (7,2) \to (7,0)$ is 5; leave; 3. $L_{Bug2} = 2 + 4 + 2 + 5 + 3 =
\mathbf{16}$. Bound: each obstacle is crossed twice ($n_i = 2$), so $10 + \tfrac12(16 + 20) =
\mathbf{28} \ge 16$. **Tangent Bug's first decision** (infinite range): from $(0,0)$ the blocking
obstacle is $\mathcal{WO}_1$ with $O_1 = (2,1)$, $O_2 = (2,-1)$; $d(x, O_i) + d(O_i, q_{goal}) =
\sqrt5 + \sqrt{65} \approx 10.298$ for both — a tie, broken by the configured turn direction.
**Tracer check:** the offset curve of $\mathcal{WO}_1$ at $W^* = 0.5$ has length $8 + \pi \approx
\mathbf{11.1416}$ (Derivation 6).

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: *a blindfolded walker with one hand on the wall.* Colors: start green, goal red,
obstacles slate, Bug paths in robot orange (three shades by lane, legend-labeled), the m-line as a thin
purple dashed path (it *is* the intended path), the offset curve and tracer samples in graph blue (a
one-dimensional roadmap in disguise, as Ch. 8 reveals). All widgets autoplay, one headline parameter
each, seeded re-roll for obstacle layouts, static fallback from the same code.

- **Widget w3.1: Bug Arena** — type: wasm-sim (flagship). Three lanes, one map, one query: Bug1, Bug2,
  Tangent Bug ($R$ finite) race in lockstep, each with its path, hit/leave points labeled as in
  C figures 2.1–2.4, and a live length counter against its printed bound. **Manipulates:** headline —
  drag the goal; disclosure: map presets (C fig. 2.1; fig. 2.4's comb; the micro-example; an
  enclosed-goal preset), Tangent Bug's range $R$, turn direction, seed re-roll of a random polygon
  field. **Observes:** Bug2 usually wins on simple obstacles; on the comb it walks the perimeter many
  times while Bug1 walks it once; on the enclosed-goal preset all three halt with "unreachable" — Bug1
  after one loop. **Misconception killed:** *"greedy is faster."* Greedy is faster on average and
  unboundedly slower in the worst case, and the counter shows both.
- **Widget w3.2: Boundary Follower** — type: interactive animation. A nonconvex obstacle; the robot at
  offset $W^*$; the tracer's steps: a tangent prediction $\Delta\lambda$ along $(\nabla D)^\perp$ (an
  arrow), the correcting line, and each Newton correction (eq. 2.5) as a shrinking hop onto the blue
  offset curve; a residual readout $|G(x_h)|$ per iteration showing the squares ($10^{-2}, 10^{-4},
  10^{-8}$). **Manipulates:** headline — $\Delta\lambda$; disclosure: $W^*$, iterations shown, a "cusp"
  preset where $W^*$ exceeds the local radius of curvature. **Observes:** small steps barely need
  correction; large steps fall off and are hauled back in two iterations; at the cusp the tracer jumps
  branches — the implicit function theorem's hypothesis failing in public. **Misconception killed:**
  *"following a wall is trivial."* It is root-finding, and its guarantee has hypotheses.
- **Widget w3.3: Tangent Scope** — type: interactive animation. The robot at $x$ with 360 thin rays of
  $\rho_R$, the intervals of continuity drawn thick on the boundaries, the $O_i$ labeled, the point $T$,
  and each candidate's heuristic $d(x, n) + d(n, q_{goal})$ printed; the chosen subgoal pulses.
  **Manipulates:** headline — the range $R$; disclosure: drag $x$ and $q_{goal}$; the C figure 2.8
  preset where the heuristic prefers $O_2$ until the robot can see that $\mathcal{WO}_2$ blocks it.
  **Observes:** as $R \to 0$ the $O_i$ collapse onto the hit point and Tangent Bug becomes Bug2-like;
  as $R$ grows the path straightens toward figure 2.13. **Misconception killed:** *"a range sensor
  gives you the map."* It gives you a few discontinuities, and that is enough.

Dashboard layout sketch: w3.1 full width at the top of the C section, three lanes side by side with a
shared scoreboard beneath (length, bound, hit/leave counts, status); w3.2 and w3.3 are half-width cards
beside §2.3.3 and §2.2 with w3.2's residual strip under its canvas; on narrow layouts the lanes stack.

## 5. Practical (P) — Rust Implementation

**Crates:** `nalgebra` 0.35; `collide` and `robots` (Ch. 2: `Scene`, `Collision`, `RangeSensor`,
`World::apartment`); `rand` 0.9 `SmallRng` for seeded obstacle fields and queries; `proptest` (dev)
for the bound tests. No planning or geometry crate beyond `parry2d` behind the Ch. 2 trait.

**Module plan:** `crates/bugs/src/{lib.rs, bug1.rs, bug2.rs, tangent.rs, scan.rs, trace.rs}` with
`trace.rs` exporting the owned artifact; `examples/two_rectangles.rs`, `examples/apartment_bugs.rs`;
`tests/bounds.rs`. Web mirror: `web/lib/plan/bugs.ts` and `web/lib/plan/trace-curve.ts`, pinned in
`lib/__checks__.ts`.

**Key types & signatures:**

```rust
use nalgebra::{Point2, Vector2};
use collide::{Collision, RangeSensor, Scene};

/// The Bugs' world view: a point robot, perfect position, a sensor. Everything else is hidden.
pub struct BugWorld<'s> { pub scene: &'s Scene, pub sensor: RangeSensor /* max_range = 0 ⇒ contact */ }
impl BugWorld<'_> {
    pub fn touching(&self, x: Point2<f64>) -> bool;      // Scene::distance of a point footprint < ε
    pub fn scan(&self, x: Point2<f64>) -> Vec<f64>;       // ρ_R(x, ·) for all rays (Ch. 2)
}

pub struct BugPath { pub points: Vec<Point2<f64>>, pub hits: Vec<Point2<f64>>, pub leaves: Vec<Point2<f64>> }
impl BugPath { pub fn length(&self) -> f64; }
pub enum BugOutcome { Reached(BugPath), Unreachable(BugPath) }
#[derive(Clone, Copy)] pub enum Turn { Left, Right }

/// C Alg. 1. Boundary following uses exact polygon walking when the scene exposes vertices and
/// `trace_curve` at W* = 0⁺ otherwise. `step` is the motion resolution.
pub fn bug1(w: &BugWorld, q_start: Point2<f64>, q_goal: Point2<f64>, step: f64, turn: Turn) -> BugOutcome;
/// C Alg. 2. Same behaviors, greedy leave rule on the fixed m-line.
pub fn bug2(w: &BugWorld, q_start: Point2<f64>, q_goal: Point2<f64>, step: f64, turn: Turn) -> BugOutcome;
/// C Alg. 3. Needs a finite-range sensor; R = ∞ allowed.
pub fn tangent_bug(w: &BugWorld, q_start: Point2<f64>, q_goal: Point2<f64>, step: f64) -> BugOutcome;

// scan.rs — C §2.2 and eq. 2.4, from a scan alone (sensor-only, no scene access).
pub struct Interval { pub from: usize, pub to: usize }           // ray indices; endpoints are the O_i
pub fn continuity_intervals(scan: &[f64], max_range: f64) -> Vec<Interval>;
pub fn distance_from_scan(scan: &[f64]) -> f64;                  // D(x) = min_s ρ(x, s)
pub fn gradient_from_scan(scan: &[f64]) -> Vector2<f64>;         // unit vector away from the argmin ray
pub fn tangent_from_scan(scan: &[f64], prev: Vector2<f64>) -> Vector2<f64>; // (∇D)^⊥, sign by continuity

// trace.rs — THE OWNED ARTIFACT (Choset App. D; eq. 2.5). Generic: Chs. 7–10 pass their own G.
pub struct TraceParams { pub step: f64, pub newton_iters: usize, pub tol: f64, pub max_steps: usize }
pub enum TraceStop { Closed, Reached(Point2<f64>), Singular, Budget }
pub struct Trace { pub points: Vec<Point2<f64>>, pub stop: TraceStop, pub residuals: Vec<f64> }
/// Predictor–corrector on the zero set of G: predict x + Δλ·t with t ⊥ ∇G, then Newton-correct
/// along ∇G on the correcting line: y ← y − G(y)/D_Y G(y). Stops when the curve closes, when
/// `stop(x)` fires, or when D_Y G ≈ 0 (Theorem D.1.1's hypothesis violated — reported, never hidden).
pub fn trace_curve(
    g: impl Fn(Point2<f64>) -> f64,
    dg: impl Fn(Point2<f64>) -> Vector2<f64>,
    x0: Point2<f64>, direction: Turn, params: TraceParams,
    stop: impl Fn(Point2<f64>) -> bool,
) -> Trace;
/// Convenience: the offset curve of a scene at W*, G = D − W*, ∇G = ∇D via `Collision::witness`.
pub fn offset_curve(scene: &Scene, w_star: f64, x0: Point2<f64>, dir: Turn, p: TraceParams) -> Trace;
```

**Worked end-to-end example** (`cargo run -p bugs --example two_rectangles`) prints

```text
Bug1   length 34.0000  bound d + 1.5·Σp = 37.0000   hits (2,0) (6,0)  leaves (4,0) (7,0)
Bug2   length 16.0000  bound d + ½·Σ n_i p_i = 28.0000   hits (2,0) (6,0)  leaves (4,0) (7,0)
TBug∞  first subgoal: tie 10.2984 between O=(2,1) and O=(2,-1) → Left
offset curve of WO1 at W*=0.5: closed after 223 steps, length 11.1416 (8 + π), max |G| 3e-10
```

Tests: `bug1_two_rectangles` asserts length 34 to $10^{-6}$ and the hit/leave points;
`bug2_two_rectangles` asserts 16; `bounds_hold` is a `proptest` over 500 seeded fields of 1–6 random
convex polygons and random queries, asserting both bounds whenever the outcome is `Reached`, and that
`Unreachable` is returned iff the goal's connected component differs from the start's (flood fill);
`tangent_first_subgoal` asserts the 10.2984 tie; `offset_square_length` asserts the traced length to
$8 + \pi$ within $10^{-3}$ and every traced point's $|G| < 10^{-8}$; `tracer_reports_singular` runs the
cusp preset and asserts `TraceStop::Singular` rather than a silent branch jump; `scan_minimum_is_D`
asserts `distance_from_scan == Scene::distance` to $10^{-6}$ on 1,000 seeded points (C problem 2.1).
`examples/apartment_bugs.rs` runs all three Bugs for Rusty-as-a-point in the Apartment inflated by
$r = 0.11$ over 100 seeded queries and prints mean lengths and bound slack.

**Runnable artifact:** the examples; in-page w3.1–w3.3 run `web/lib/plan/bugs.ts` and `trace-curve.ts`,
whose `__checks__.ts` invariants are the same 34, 16, and $8 + \pi$. The integration lab is
`apartment_bugs`: pick a seed, watch Bug1 crawl the Apartment from the range sensor alone, read the slack.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w3.1 | Bug Arena | wasm-sim | `bugs` + `collide` (Ch. 2); `web/lib/plan/bugs.ts`; seeded `Rng` | drag goal; map presets; $R$ slider; turn direction; seed re-roll | exhaustive vs. greedy; bounds hold; where Bug2 loses |
| w3.2 | Boundary Follower | interactive animation | `bugs::trace_curve`; `web/lib/plan/trace-curve.ts` | $\Delta\lambda$ slider; $W^*$; iteration stepper; cusp preset | predictor–corrector; quadratic convergence; IFT hypothesis failing |
| w3.3 | Tangent Scope | interactive animation | `bugs::scan` + `collide::RangeSensor` | $R$ slider; drag $x$, $q_{goal}$; fig. 2.8 preset | intervals of continuity; $O_i$; the heuristic and its blind spot |
| — | Three-lane race hook | animation (autoplay) | `bugs` replay | none | Bug2 wins, then loses (chapter hook) |

## 7. Exercises & Extensions

1. **(F)** Prove that in Bug1 the robot never contacts an obstacle lying entirely outside the disk of
   radius $d(q_{start}, q_{goal})$ about $q_{goal}$ (C problem 2.7), and use it to tighten the sum in
   eq. (2.1) to obstacles meeting that disk.
2. **(F)** Write out $D_Y G$ for planar boundary following at offset $W^*$ (C problem 2.11) and show
   that $G_1 = D + 1$ and $G_2 = D + 2$ have the same Jacobian (C problem 2.12). State the exact
   hypothesis of Theorem D.2.1 that fails at a point of the medial axis.
3. **(C — predict, then verify with w3.1)** Load the comb preset. Before pressing play, estimate $n_1$
   and predict Bug2's length to within 20 % from eq. (2.2). Verify; then rotate the goal so the comb's
   teeth are parallel to the m-line and predict which Bug wins now.
4. **(C — w3.2)** Set $\Delta\lambda$ to half the obstacle's smallest radius of curvature. Predict how
   many Newton iterations reach $|G| < 10^{-8}$ from the quadratic rate; verify. Then raise $W^*$ past
   that radius and explain, via Theorem D.1.1, what the tracer did.
5. **(P)** Implement a limited-field-of-view Tangent Bug (C problem 2.10): the sensor sees a $\pm 90°$
   wedge about the heading. Which $O_i$ can it still find, and which bookkeeping distance becomes
   unsafe? Add a lane to w3.1 and compare lengths over 100 seeds.
6. **(P, stretch)** Give Rusty the sister book's noisy LiDAR and odometry drift instead of the ideal
   sensor and perfect position (C problem 2.6), and make Bug1 *robust*: a tolerance for "re-encounter
   $q^H_i$," a hysteresis band for the leave test. Measure the failure rate vs. noise over 200 seeds
   and report the smallest tolerance that keeps the bound within 10 %.

## 8. Modernization Notes

- **Kept from Choset, and why.** All three algorithms with Choset's numbering and statements, both
  bounds with proofs, the exhaustive-vs-greedy reading, and §2.3's three questions (what information,
  how to infer it, how to use it) with the continuation-method answer. The Bugs are kept not as
  practical navigation but as the shortest honest route to a complete sensor-based planner *and* as
  the chapter where the curve tracer is born. App. D's two theorems are stated in full because the
  book reuses them four more times.
- **Added beyond 2005.** (i) The micro-example with exact lengths and a property test of both bounds
  over seeded random worlds — Choset states the inequalities; here they are asserted thousands of
  times. (ii) `trace_curve` as a generic artifact with explicit singularity reporting; Choset's
  predictor–corrector is prose. (iii) The $p + 2\pi W^*$ check that makes the tracer's correctness a
  number. (iv) An empirical "which Bug wins where" lane in the spirit of comparative studies (Ng &
  Bräunl 2007, verify). (v) The honest cross-link that production stacks (Nav2 local planners, Ch. 19's
  MPC) have replaced Bug-style navigation, while the two-behavior architecture survives in every
  recovery behavior.
- **Dropped/condensed.** §2.3.2's ultrasonic physics, beam patterns, and centerline model become a
  one-paragraph cross-link to the sister book's Chs. 4 and 10; figures 2.11–2.13 become w3.3's $R$ slider.
- **Honesty items the prose must keep.** The Bugs assume a point robot with perfect positioning in a
  bounded world with finitely many obstacles; Tangent Bug's completeness is left as C problem 2.9 and
  the text says so. The tracer "hovers" around the curve (C figure 2.20): it produces samples, not the
  curve, and its guarantee is local, conditioned on $D_Y G \ne 0$ — the cusp preset exists to make
  that failure visible.
