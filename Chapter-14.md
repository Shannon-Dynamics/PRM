# Chapter 14 — Planning for Many Bodies: Multi-Robot, Manipulation, and Kinodynamic Previews

> Part III — Sampling-Based Planning · Estimated length: 9 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Three chapters of sampling-based planning assumed one robot, one query, one rigid notion of
"free". This chapter removes those assumptions one at a time and finds the machinery does not care:
two robots' configuration space is the product of theirs; a manipulated object is an extra body
sometimes free and sometimes rigidly tied to the hand; a car that cannot be steered along a
straight line can still grow a tree by *simulating* instead of steering. The "aha": **every planner
in Part III runs unchanged on a composite configuration space — only the price changes, and it is
exponential in the number of bodies, so many-body planning is about when to refuse to pay it:
decoupling, prioritization, modes, and conflict-based search.** Choset's §7.5 surveys these
extensions in ten pages; this chapter gives multi-robot and manipulation planning full treatment,
adds the two modern results that changed practice (conflict-based search, the task-and-motion
view), previews forward propagation in one page as Hitch's first planner, and names the systems
(OMPL, MoveIt) in which all of Part III is infrastructure.

Story line:
1. **Hook:** two Rustys in an Apartment corridor, each planned alone with Ch. 11's PRM. Both paths
   are collision-free. Executed together, they meet head-on at the pillar (autoplay).
2. **Play (C):** w14.1 Two-Robot Composite C-space — drag one disc, watch the other's free space
   lose a moving hole of radius $r_1+r_2$; flip between prioritized and coupled plans.
3. **Formalize (F):** the product manifold and its two kinds of C-obstacle; centralized vs.
   decoupled vs. prioritized planning and the incompleteness of the last two; conflict-based search
   stated and proved optimal; transit/transfer modes and the manipulation graph; Theorem 7.4.3
   applied to a steer-free local planner.
4. **Algorithms and preview:** composite PRM/RRT on `Composite<A,B>`, prioritized planning in
   $\mathcal{Q}_i\times[0,T]$, Choset's one-robot-at-a-time extension, CBS, the two-level
   manipulation PRM; then one page on "plan with a simulator instead of a steer" — Hitch's Ch. 2
   kinematic update as a black box, nothing derived (Ch. 21 does); OMPL and MoveIt as the systems
   in which all of Part III is infrastructure (verify).
5. **Implementation (P):** `multi` crate — `Composite`, `MultiBody`, `prioritized`, `cbs`,
   `manip`, `propagate`; the two-disc collision test pinned by a unit test.
6. **Integration lab & pointer box:** three Rustys under prioritized, coupled and CBS plans; Reach
   moving a block; Hitch's steer-free tree in the Lot; assembly, flexible objects and biology as a
   pointer box; close on what Part IV adds — the robot does not know where it is.

## 2. Prerequisites & Position

- **Builds on:** Ch. 2 (`Collision` — robot-robot queries are the same parry2d call as
  robot-obstacle; Hitch's kinematic update used *only* as a black-box simulator; Reach's FK for
  grasp frames), Ch. 4 (a disc-disc C-obstacle is a disc of radius $r_1+r_2$), Ch. 5 (`Manifold` —
  this chapter adds the product), Ch. 6 (grid A\* is CBS's low level and the coordination search),
  Ch. 11 (`Prm`, `Sampler`, `Steer`, `FreeSpace`, Theorem 7.4.3's asymmetric $R$), Ch. 12 (`Tree`,
  `Extend`, `Rrt` — the propagated tree keeps `Tree`, drops `Extend`), Ch. 13 (`Anytime`).
- **Feeds into:** Ch. 21 (forward propagation in full, kinodynamic RRT/RRT\*, Dubins as the steer
  that makes Hitch's tree good), Ch. 23 (three robots at once; the capstone's stack mapped onto
  OMPL/MoveIt/Nav2 via this chapter's table), Ch. 19 (trajectory optimization of transfer paths).
- **Baseline sources:** C §7.5 intro (b. 253), §7.5.1 (b. 253–254: control-based planning —
  *preview only*; full in Ch. 21), §7.5.2 (b. 254–257: centralized, decoupled, prioritized, the
  one-robot-at-a-time extension, SRT for multiple robots), §7.5.3 (b. 257–259: transit/transfer,
  regrasping, two-level FuzzyPRM, aSyMov, nonprehensile manipulation), §7.5.4–7.5.6 (b. 259–267:
  pointer box). Sister book: Ch. 23 (`…/chapters/ch23-mppi`, rollouts through a simulator — the
  same move as forward propagation), Ch. 20 (`…/chapters/ch20-motion-planning`), Ch. 4
  (`…/chapters/ch04-rusty-and-sensors`, the simulator contract). Modernization set (all **to
  verify**): Sharon, Stern, Felner & Sturtevant, CBS (AIJ 219, 2015); Garrett et al., TAMP survey
  (Annu. Rev. Control Robot. Auton. Syst. 4, 2021); Şucan, Moll & Kavraki, OMPL (IEEE RAM 19(4),
  2012); MoveIt (canonical paper to verify). Kinodynamic RRT is *classical* [C ref. 270, 271].

## 3. Foundation (F) — Mathematical Core

**Notation introduced** (chapter-scoped):

| Symbol | Meaning |
|---|---|
| $\mathcal{Q}=\mathcal{Q}_1\times\dots\times\mathcal{Q}_m$, $q=(q_1,\dots,q_m)$ | composite configuration space of $m$ bodies; $\dim\mathcal{Q}=\sum_i\dim\mathcal{Q}_i$ |
| $\mathcal{QO}_i^{\mathcal{W}}$, $\mathcal{QO}_{ij}$ | robot–obstacle C-obstacle of body $i$ (lifted to $\mathcal{Q}$); robot–robot C-obstacle $\{q: R_i(q_i)\cap R_j(q_j)\ne\emptyset\}$ |
| $c_i:[0,1]\to\mathcal{Q}_{free,i}$, $(s_1,\dots,s_m)\in[0,1]^m$ | individual paths; coordination space of path parameters |
| $\mathcal{Q}_i\times[0,T]$ | configuration–time space for prioritized planning |
| $(a_i,a_j,v,t)$, $(a_i,v,t)$, $N$, $\mathrm{SIC}(N)$ | CBS conflict; constraint; constraint-tree node; its sum of individual costs |
| $\mathcal{G}$, $P$, $\mathcal{M}_{transit}(p)$, $\mathcal{M}_{transfer}(g)$ | grasp set; stable placements; transit manifold (object fixed at $p$); transfer manifold (object rigidly attached by $g$) |
| $MG$ | manipulation graph: nodes $(p,g)$, edges transit/transfer paths |
| $f:\mathcal{Q}\times U\to\mathcal{Q}$, $U$, $\Delta t$ | Choset's control function (state propagation); control set; propagation interval |

**Definitions:**
- *Composite free space.* $\mathcal{Q}_{free}=\bigcap_i\pi_i^{-1}(\mathcal{Q}_{free,i})\setminus
  \bigcup_{i<j}\mathcal{QO}_{ij}$ — individual free spaces lifted, minus pairwise interference.
- *Decoupled planning.* Plan each $c_i$ in $\mathcal{Q}_{free,i}$ alone, then coordinate velocities
  along the fixed paths [C ref. 219, Kant & Zucker]; *prioritized* planning plans robots in order,
  each treating earlier robots as moving obstacles.
- *Coordination diagram.* The subset of $[0,1]^m$ where some pair $(c_i(s_i),c_j(s_j))$ collides;
  a monotone free path from $\mathbf{0}$ to $\mathbf{1}$ is a valid velocity schedule.
- *Conflict / constraint* (CBS). A conflict: agents $a_i,a_j$ occupy vertex $v$ (or swap an edge)
  at time $t$; a constraint $(a_i,v,t)$ forbids it for one agent.
- *Transit / transfer path* (C §7.5.3). Robot motion with the object at rest at a placement / with
  the object grasped; mode switches only at $\mathcal{M}_{transit}(p)\cap\mathcal{M}_{transfer}(g)$.
- *Forward propagation.* Producing $q'=f(q,u)$ by applying a control for $\Delta t$ instead of
  steering to a chosen $q'$; the relation $qRq'$ is asymmetric.

**Derivations / theorems:**

1. **Structure and price of the composite C-obstacle.** *Statement:* for discs of radii $r_i,r_j$,
   $\mathcal{QO}_{ij}=\{q:\|p_i-p_j\|<r_i+r_j\}$ is a cylinder over a ball in the difference
   coordinate — convex in $p_i-p_j$, of full dimension in $\mathcal{Q}$; and Theorem 7.4.1's bound
   with $d=\sum_i d_i$ requires samples growing like $(1/\rho)^{d}$ to tile a path of clearance
   $\rho$. *Sketch (4 steps):* (i) $R_i(q_i)\cap R_j(q_j)\ne\emptyset$ iff center distance
   $<r_i+r_j$ (Ch. 4's Minkowski argument applied to two moving bodies); (ii) the set depends on
   $q$ only through $p_i-p_j$, hence the cylinder; (iii) lifting $\mathcal{QO}_i^{\mathcal{W}}$
   gives cylinders in the other direction, so $\mathcal{Q}_{free}$ is their complement; (iv) in
   Theorem 7.4.1, $\sigma\rho^d$ shrinks geometrically in $d$ at fixed $\rho$, which is the
   exponential price. *Collapsible:* polygons via Ch. 4's star algorithm on $R_i\oplus(-R_j)$;
   why a coupled path's clearance is typically smaller than any individual path's.
2. **Decoupled and prioritized planning are incomplete.** *Statement:* there are instances with a
   coupled solution where no velocity coordination of independently planned paths succeeds, and
   where prioritized planning fails for every priority order. *Sketch (4 steps):* (i) corridor with
   one alcove, two robots that must swap ends; (ii) each individually shortest path is the corridor
   axis, so the coordination diagram's obstacle is the full anti-diagonal band — no monotone path
   from $\mathbf 0$ to $\mathbf 1$ exists; (iii) with priority, robot 1's path occupies the corridor
   at all times robot 2 needs it, and vice versa; (iv) the coupled planner finds "robot 1 waits in
   the alcove", which neither decoupled stage can express. *Collapsible:* why decoupling stays useful
   (dimension never grows); Choset's remark that completeness is lost "even when the algorithms
   used in both stages are complete"; CBS's fix — re-plan *paths*, not just velocities.
3. **Conflict-based search is complete and optimal for sum-of-costs (Sharon et al. 2015)
   (verify).** *Statement:* on a finite graph with unit-time moves, CBS returns a conflict-free
   joint plan minimizing $\mathrm{SIC}$ whenever one exists. *Sketch (6 steps):* (i) the low level
   plans each agent optimally by Ch. 6 A\* over (vertex, time) under its constraints; (ii) the high
   level is best-first on $\mathrm{SIC}$ over a binary constraint tree; (iii) at node $N$ find the
   first conflict; if none, return; (iv) split into children adding $(a_i,v,t)$ and $(a_j,v,t)$,
   re-planning only the constrained agent; (v) any valid solution satisfies one child's constraint,
   so no optimal solution is pruned; (vi) costs are non-decreasing and integer down the tree, so the
   first conflict-free node popped is optimal and a solution is eventually found. *Collapsible:* edge conflicts; the exponential worst
   case; why CBS beats coupled A\* when conflicts are few — the regime of robots in an apartment.
4. **A manipulation plan is a path in a union of lower-dimensional manifolds.** *Statement:* in
   the composite space of robot and object, feasible motions lie in
   $\bigcup_p\mathcal{M}_{transit}(p)\cup\bigcup_{g\in\mathcal{G}}\mathcal{M}_{transfer}(g)$, each
   of dimension $\dim\mathcal{Q}_{robot}$, and the problem is connectivity of the manipulation
   graph $MG$ whose nodes are intersections $(p,g)$. *Sketch (4 steps):* (i) an unheld object at a
   stable placement cannot move, so the object coordinates are frozen: a transit manifold; (ii) a
   held object's configuration is a fixed rigid transform of the hand's: a transfer manifold;
   (iii) switching requires both — a grasp $g$ at a placement $p$ — the intersection; (iv) a
   sequence transit–grasp–transfer–release–transit is a path through $MG$; regrasping is a cycle
   through two grasps at one placement. *Collapsible:* two-level FuzzyPRM [C ref. 335, Nielsen &
   Kavraki] with edge probabilities and lazy verification; aSyMov [C ref. 170] as proto-TAMP; the
   TAMP survey (Garrett et al. 2021) (verify): symbolic actions ↔ modes, parameters ↔ grasps.
5. **Steer-free planners inherit Theorem 7.4.3.** *Statement:* with $R=\{(q,q'):\exists u\in U,\
   q'=f(q,u)\text{ and the propagated segment is free}\}$, Choset's abstract path tiling applies
   verbatim: probabilistic completeness holds iff the random incremental algorithm succeeds with
   nonzero probability, and then failure decays exponentially. *Sketch (3 steps):* (i) $R$ is
   measurable for a continuous $f$; (ii) the proof of Theorem 7.4.3 never used symmetry or
   reflexivity; (iii) sampling controls instead of configurations is a change of the measure
   $\mu$, which the theorem also allows. *Collapsible:* the preview stops here — what $f$ is for Hitch,
   why it cannot be inverted into a steer, and controllability are Chapters 20–21.

**Named algorithms:**

| Algorithm | Signature | Complexity |
|---|---|---|
| Composite PRM / RRT (Chs. 11–12 on `Composite`) | `(Composite<A,B>, MultiBody, …)` | as Chs. 11–12 with $d=\sum d_i$; pairwise checks $O(m^2)$ per sample |
| Prioritized planning | `([robots], order) -> [Trajectory]` | $m$ single-robot plans in $\mathcal{Q}_i\times[0,T]$; incomplete |
| One-robot-at-a-time extension (C §7.5.2, [C ref. 14]) | `(T, q_rand) -> Option<q_new>` | $m$ sequential extensions with cumulative collision checks |
| CBS (verify) | `(grid, starts, goals) -> Option<[Path]>` | low level $O(|V|T\log)$ per re-plan; high level exponential worst case in conflicts |
| Two-level manipulation PRM [C ref. 335] | `(placements, grasps) -> MG + motion plans` | upper level over $|P|\cdot|\mathcal{G}|$ nodes; lower level Ch. 11 PRM per edge, lazily |
| Forward-propagated RRT (classical, [C ref. 270, 271]; preview) | `(q0, f, U, Δt, n) -> Tree` | per iteration: NN + $|U_{sampled}|$ propagations; no steer |

**Numeric micro-example** (the contract with the unit test in §5). Two Rusty discs of radius
$r_1=r_2=0.5$ in a corridor $y\in[0,2]$ (walls at $y=0$ and $y=2$), composite
$q=(x_1,y_1,x_2,y_2)\in\mathbb{R}^4$.
- **Test $q=(1.0,1.0,1.8,1.4)$.** Robot–obstacle: disc 1's clearance to both walls is
  $\min(1.0-0.5,\ 2-1.0-0.5)=0.5>0$; disc 2 spans $y\in[0.9,1.9]\subset(0,2)$ — both individually
  free. Robot–robot: $\|p_1-p_2\|=\sqrt{0.8^2+0.4^2}=\sqrt{0.80}=0.8944<r_1+r_2=1.0$ →
  $q\in\mathcal{QO}_{12}$: **collision**, penetration $1.0-0.8944=0.1056$. Each robot's own check
  passes; only the composite check catches it — the hook, in numbers.
- **Test $q'=(1.0,1.0,2.0,1.4)$.** $\|p_1-p_2\|=\sqrt{1.0^2+0.4^2}=\sqrt{1.16}=1.0770>1.0$ → **free**.
- **Boundary along the straight edge $q'\to q$** (only $x_2$ moves, $2.0\to1.8$): the edge enters
  $\mathcal{QO}_{12}$ where $(x_2-1)^2+0.16=1$, i.e. $x_2=1+\sqrt{0.84}=1.9165$; the Ch. 11
  subdivision $\Delta$ rejects it at its first midpoint $x_2=1.9$ ($\|p_1-p_2\|=0.9849<1$).
- **Prioritized slice.** With robot 1 fixed at $(1.0,1.0)$, robot 2's slice C-obstacle is the disc
  of radius $1.0$ about $(1.0,1.0)$, spanning $y\in[0,2]$ — the full corridor height at $x_2=1.0$:
  robot 2 must wait or fail, derivation F.2 in numbers.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: **"one space, many bodies, and a bill that grows exponentially — pay it only when
you must."** Colors: robots and executed paths **orange** (robot 2 dashed); robot–robot C-obstacle
**amber hatched** (a moving C-obstacle), walls **slate**, trees and roadmaps **blue**, solution
**purple**, start **green**, goal **red**; the block is slate at rest and orange when held — a
visual statement of the two modes.

- **Widget w14.1: Two-Robot Composite C-space** — type: wasm-sim (flagship). Two Rusty discs in a
  corridor with one alcove. Three panels: the workspace; robot 2's 2D slice of the 4D C-space with
  the amber disc of radius $r_1+r_2$ that follows robot 1; and the coordination diagram $[0,1]^2$
  for the two individually planned paths with its collision region. **Manipulates:** headline —
  corridor width (disc radii; the swap problem appears below $4$); disclosure — planner chips
  (coupled PRM on `Composite`, prioritized 1→2 or 2→1, coordination schedule), drag either robot,
  seed. **Observes:** the amber hole
  moving; the diagram's monotone path appearing or cut off; prioritized succeeding in one order and
  failing in the other; the coupled planner's sample count vs. width. **Misconception killed:** *"plan each robot separately and add a wait."* — the swap
  problem has no wait that works; the alcove detour is only expressible in the composite space.
- **Widget w14.2: Pick-and-Place Modes** — type: wasm-sim. Reach on the Workbench moves a block
  between two placements. Left: the workspace; right: the manipulation graph $MG$ with nodes
  $(p,g)$ and two overlaid torus pictures — the transit manifold's amber C-obstacle (block at rest
  is an obstacle) and the transfer manifold's (block attached to the hand changes Reach's shape).
  **Manipulates:** headline — number of grasps $|\mathcal{G}|$ (1 … 6); disclosure — a third
  placement, lazy edge verification (FuzzyPRM-style), seed. **Observes:** with one grasp the graph
  may be disconnected (regrasping is impossible); edges verified lazily along the chosen path; the
  block switching slate ↔ orange as it is released and held. **Misconception killed:** *"manipulation planning is path planning
  with a bigger robot."* — it is a union of lower-dimensional manifolds, and switching happens only
  at their intersections.
- **Widget w14.3: Hitch Without a Steer** — type: wasm-sim. Forward-propagated RRT for the car in
  the Lot: each extension samples controls, integrates Hitch's Ch. 2 kinematic update for $\Delta t$,
  and keeps the result closest to $q_{rand}$. The Ch. 12 straight-line "RRT" is ghosted in gray —
  undrivable. **Manipulates:** headline — number of control samples per extension (1 … 20);
  disclosure — $\Delta t$, goal bias, seed. **Observes:** a tree of drivable arcs; wasted
  propagations; time to the goal region; a caption saying the simulator is a black box here. **Misconception
  killed:** *"an RRT needs a steering function."*
- **Widget w14.4: Constraint Tree** — type: interactive animation. CBS on a small grid with 2–5
  Rustys. **Manipulates:** headline — number of agents; disclosure — step through the tree, show the
  coupled-A\* joint-state count. **Observes:** the first conflict; two
  children with one constraint each; $\mathrm{SIC}$ per node; the joint-state count exploding while
  the tree stays small. **Misconception killed:** *"optimal
  multi-agent paths require searching the joint space."*

Dashboard layout: w14.1 full-width at the top of C with the coordination diagram docked right;
w14.2 full-width in the manipulation subsection; w14.3 half-width in the boxed one-page preview;
w14.4 half-width beside derivation F.3. All seeded, seed shown, static SVG fallbacks.

## 5. Practical (P) — Rust Implementation

**Crates:** `sampling` (Chs. 11–13), `cspace` (Ch. 5 `Manifold`), `collide` + `robots` + `sim`
(Ch. 2: pairwise `Collision`, Hitch's kinematic update, the worlds), `search` (Ch. 6 A\*),
`nalgebra` 0.35, `petgraph` 0.8 (manipulation graph), `rand` 0.9 `SmallRng`, `pathfinding` 4.15
(cross-check of the time-expanded A\*). No new external dependency.

**Module plan:** `crates/multi/` — `src/composite.rs` (`Composite<A,B>: Manifold`, `Product<M,
const N>`; `Chart` impl so the Ch. 11 kd-tree works); `src/body.rs` (`MultiBody<M>: FreeSpace<M>`,
separate world/pair counters); `src/prioritized.rs` (`TimeExtended<M>` = `Composite<M, Interval>`,
`MovingObstacle`, `plan_prioritized(robots, order, horizon) -> Result<Vec<Trajectory>, usize>`);
`src/coordination.rs` (two-path diagram, grid A\*); `src/cbs.rs` (`Conflict`, `Constraint`,
`CtNode`, `Cbs`); `src/manip.rs` (`Mode::{Transit{placement}, Transfer{grasp}}`,
`ManipulationGraph` over `petgraph::UnGraph<(p, g), ModeEdge>`, `TwoLevelPrm` with lazy edges);
`src/propagate.rs` (`trait Propagate`, `KinoRrtPreview` over Ch. 12's `Tree`);
`examples/{two_discs,three_rustys,hitch_no_steer}.rs`.

**Key types & signatures** (compiles in spirit; everything from Chs. 2, 5, 6, 11–13 imported):

```rust
use manifold::Manifold;
use sampling::{FreeSpace, Chart, tree::{Tree, NodeId}};
use rand::rngs::SmallRng;

/// Q = A × B. dist is the L2 combination so Theorem 7.4.1's balls are balls; interpolate is
/// componentwise, which is why Ch. 11's straight-line Steer works unchanged on the product.
pub struct Composite<A: Manifold, B: Manifold> { pub a: A, pub b: B }
impl<A: Manifold, B: Manifold> Manifold for Composite<A, B> {
    type Point = (A::Point, B::Point);
    fn dim(&self) -> usize { self.a.dim() + self.b.dim() }
    fn dist(&self, p: &Self::Point, q: &Self::Point) -> f64 { self.a.dist(&p.0, &q.0).hypot(self.b.dist(&p.1, &q.1)) }
    // interpolate: componentwise; sample: (a.sample, b.sample)
}

/// Free space of m bodies: each body against the world (Ch. 2), then every pair. Two counters,
/// because the lab reports how much of the bill is pairwise. Implements Ch. 11's FreeSpace<M>.
pub struct MultiBody<'w, M: Manifold> { pub bodies: Vec<Body<'w>>, pub space: M, pub pair_checks: u64, pub world_checks: u64 }
pub fn discs_collide(p1: [f64; 2], r1: f64, p2: [f64; 2], r2: f64) -> (bool, f64 /* center dist */);

/// Conflict-based search on a grid; low level is Ch. 6 A* over (cell, time) with constraints.
pub struct Conflict { pub a: usize, pub b: usize, pub cell: GridIdx, pub t: u32, pub edge: Option<GridIdx> }
pub struct Constraint { pub agent: usize, pub cell: GridIdx, pub t: u32 }
pub struct Cbs<'g> { grid: &'g search::Grid, pub expansions: u64 }
impl<'g> Cbs<'g> {
    pub fn solve(&mut self, starts: &[GridIdx], goals: &[GridIdx]) -> Option<Vec<Vec<GridIdx>>>;
}

/// PREVIEW (Ch. 21 owns the full version). A simulator step; Hitch's Ch. 2 kinematic update
/// implements it as a black box. Nothing about the dynamics is derived or inspected here.
pub trait Propagate<M: Manifold> {
    type Control;
    fn propagate(&self, q: &M::Point, u: &Self::Control, dt: f64) -> M::Point;
}
pub struct KinoRrtPreview<M: Chart, P: Propagate<M>> {
    pub tree: Tree<M>, pub sim: P, pub controls_per_extension: usize, pub dt: f64, pub rng: SmallRng,
}
impl<M: Chart, P: Propagate<M>> KinoRrtPreview<M, P> {
    /// Sample q_rand; nearest; sample k controls; propagate each; keep the free result closest
    /// to q_rand (LaValle & Kuffner's rule). No Steer, no Extend: this is why it exists.
    pub fn extend_once(&mut self, free: &dyn FreeSpace<M>, controls: &mut dyn FnMut(&mut SmallRng) -> P::Control) -> Option<NodeId>;
}
```

**Worked end-to-end example:** `cargo run --example two_discs -p multi` builds the corridor, two
discs of radius $0.5$, and prints `discs_collide` for $q$ (`0.8944, collide, penetration 0.1056`)
and $q'$ (`1.0770, free`), the subdivision rejection of the edge $q'\to q$ at $x_2=1.9$
(`0.9849`), the analytic boundary `1.9165`, and robot 2's blocked slice at $x_2=1.0$.
`#[test] fn reproduces_two_disc_test()` asserts all five numbers to $10^{-4}$ and that
`MultiBody::is_free` agrees with `discs_collide` on 1 000 seeded composite configurations (parry2d
vs. closed form). `three_rustys` runs prioritized (both orders), coupled PRM on `Product<R2, 3>` and CBS on the
inflated Apartment grid (success, makespan, $\mathrm{SIC}$, checks — snapshot test, seed 14);
`hitch_no_steer` grows `KinoRrtPreview` in the Lot and asserts the goal region within 4 000
iterations (seed 14).

**Runnable artifact:** the widgets run `web/lib/plan/{composite,multibody,prioritized,cbs,manip,
propagate}.ts`; `lib/__checks__.ts` replays the two-disc numbers. The Integration lab puts three
Rustys in the Apartment (prioritized succeeds in one order, CBS always, coupled PRM at ten times
the samples), Reach moving a block on the Workbench through two grasps, and Hitch's steer-free tree
in the Lot — the last labelled "preview: Chapter 21 makes this good".

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w14.1 | Two-Robot Composite C-space | wasm-sim | `multi` → `web/lib/plan/{composite,multibody,prioritized}.ts` + `sim` corridor | corridor width, planner mode chips, drag robots, seed | product C-space; moving C-obstacle; prioritized/decoupled incompleteness |
| w14.2 | Pick-and-Place Modes | wasm-sim | `multi::manip` → `web/lib/plan/manip.ts` + `robots` Reach FK + `cspace` torus | $|\mathcal{G}|$, third placement, lazy edges, seed | transit/transfer manifolds; the manipulation graph |
| w14.3 | Hitch Without a Steer | wasm-sim | `multi::propagate` → `web/lib/plan/propagate.ts` + `robots` Hitch kinematics (black box) | controls per extension, $\Delta t$, goal bias, seed | forward propagation replaces steering (preview) |
| w14.4 | Constraint Tree | interactive animation | `web/lib/plan/cbs.ts` + Ch. 6 grid A\* | agent count, step through CT | CBS: conflicts, constraints, SIC; joint space avoided |
| f14.5 | Pointer box: assembly, flexible objects, biology | static-svg | build-time plotters | hover → one-paragraph summary each | where §7.5.4–7.5.6 went and why |
| — | Two Rustys collide (hook) | animation (autoplay) | `multi` + `sim` | replay | individually free paths are not jointly free |

## 7. Exercises & Extensions

1. **(F)** For two unit-radius discs, compute $\mu(\mathcal{QO}_{12}\cap[0,10]^4)$ exactly, compare
   with $\mu(\mathcal{QO}^{\mathcal{W}}_1)$ for a $1\times1$ pillar, and say which dominates as $m$ grows.
2. **(F)** Prove that CBS's constraint tree, with vertex and edge conflicts, never prunes a
   conflict-free solution (F.3 step (v)); give a 2-agent instance needing depth $\ge3$.
3. **(C, predict-then-verify)** In w14.1 at corridor width $3.0$ radii, prioritized order 1→2
   succeeds and 2→1 fails. Predict, *before* dragging, the width at which both orders fail while the
   coupled planner succeeds; verify, and read the coordination diagram there.
4. **(C)** In w14.2 with $|\mathcal{G}|=1$ the manipulation graph is disconnected for the
   back-to-front flip. Predict the minimum $|\mathcal{G}|$ that connects it and which node $(p,g)$
   the path uses; verify, then switch lazy verification off and count the extra edges checked.
5. **(P)** Implement Choset's one-robot-at-a-time extension (C §7.5.2) as an `Extend` impl on
   `Product<R2, N>`; compare its coupled-RRT success rate with the componentwise extension over 50
   seeds in a 4-Rusty Apartment.
6. **(P, stretch)** Add edge conflicts and a makespan objective to `Cbs`; benchmark against coupled
   A\* on 3–6 agents in the inflated Apartment grid, plotting expansions vs. joint-state count.

## 8. Modernization Notes

- **Kept from Choset:** §7.5.2 in full — centralized/decoupled/prioritized, the incompleteness
  remark, the one-robot-at-a-time extension [C ref. 14], SRT for multiple robots; §7.5.3 in full —
  transit/transfer, regrasping, two-level FuzzyPRM [C ref. 335, Nielsen & Kavraki], aSyMov [C ref.
  170]; §7.5.1 as a one-page preview in Choset's own framing (controls $U$, control function $f$,
  forward propagation) with its classical sources [C ref. 270, 271]; §7.5.4–7.5.6 as a pointer box.
- **Added beyond 2005:** conflict-based search (Sharon, Stern, Felner & Sturtevant, AIJ 2015)
  (verify) as the answer to the decoupled/coupled dilemma; the task-and-motion view (Garrett et al.
  2021) (verify) generalizing the manipulation graph; OMPL (Şucan, Moll & Kavraki, IEEE RAM 2012)
  (verify) mapped onto this book's traits — StateSpace ↔ `Manifold`, StateSampler ↔ `Sampler`,
  MotionValidator ↔ `Steer` + `EdgeCheck`, Planner ↔ `Prm`/`Rrt`/`RrtStar`; MoveIt (paper to
  verify) as the planning scene and adapters around OMPL; the sister book's MPPI as the same
  simulator-rollout move in a different costume.
- **Dropped or condensed:** nonprehensile manipulation (parts feeding, squeeze grasps, programmable
  vector fields — a paragraph with pointers), the NonDirectional Blocking Graph, flexible-object
  parametrization, folding and docking (hover paragraphs in f14.5); parallel SRT (a sentence).
- **Honesty items the prose must keep:** prioritized and decoupled planning are incomplete and
  the widget shows the failing order; CBS optimality is for the discretized grid with unit moves,
  not the continuous composite space; the composite price is exponential and the lab reports sample
  counts rather than hiding them; the preview derives nothing about Hitch's dynamics or
  controllability — the simulator is a black box and the widget caption says so; the manipulation
  graph assumes finite grasp and placement sets; the OMPL/MoveIt mapping is interface-level, to be verified.
