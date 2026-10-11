# Chapter 2 — The Lab: Three Robots, Three Worlds, One Simulator

> Part I — Foundations: Robots, Worlds, and Configuration Space · Estimated length: 8 web pages · Difficulty: Intermediate

## 1. Purpose & Story Arc

This chapter builds the laboratory the other twenty-one chapters live in: Rusty the disc/differential-
drive rover (ported from the sister book, never re-derived), Reach the planar 2R arm (3R variant),
Hitch the car with an optional trailer; the Apartment (ported), the Workbench, and the Lot; a typed
notion of *configuration* for each robot; Reach's forward kinematics; a `Collision` trait that answers
the four questions every planner asks of geometry (is $R(q)$ in collision, how far from it, where
exactly, is a swept motion safe) through `parry2d` as a sealed black box; the deterministic range
sensor Ch. 3's Bugs need; and the widget framework. Two ideas carry it. The pedagogical one: **a
configuration is a typed value, and $R(q)$ is a function** — the compiler will not let a torus point
be used where a rigid-body pose is expected, and that refusal is the first theorem of Ch. 5 enforced
by `rustc`. The engineering one, inherited: **determinism is a feature** — (seed, inputs) ↦
bit-identical runs, native and in the browser. The "aha": after ten minutes of dragging Reach's joints
while a collision flash fires, the reader has *felt* that the arm is two angles and the world is a
function of them. The Practical section is the book's architectural contract: workspace layout, crate
boundaries, type names, and widget chrome are fixed here and never renegotiated.

Story line:

1. **Hook** — w2.1 autoplays: Rusty circles the Apartment's corridor, Reach sweeps the Workbench,
   Hitch backs through the Lot; each flashes on contact. The reader takes over and makes all three collide.
2. **Play (C)** — w2.2 Query Anatomy: freeze a scene, ask the four questions, see the answers drawn.
3. **Formalize (F)** — configurations as typed values: $\mathbb{R}^2$ for the disc, $T^2$ for Reach,
   $SE(2)$ for a planar body and Hitch, $SE(2) \times S^1$ with a trailer; $R(q)$ written out for
   each; Reach's forward kinematics $\varphi$ (C §3.8) with its worked number.
4. **Algorithm** — what a collision checker computes: intersection, distance, witness points, swept
   queries — stated as a contract, internals deferred to Ch. 4 and Appendix C.
5. **Implementation (P)** — the `core` / `sim` / `robots` / `collide` / `widget-kit` crates and their
   `web/lib/` mirrors; the `Collision` trait over `parry2d`; the ported `World`, `Rusty`, `Pose2`, `Rng`.
6. **Integration lab** — one deterministic 60 s run of all three robots, replayed bit-for-bit; the
   chapter's worked example locked by tests; the seed re-roll demonstrated.
7. **Close** — the anatomy of a book widget and the chrome contract; which chapter opens which box.

## 2. Prerequisites & Position

- **Builds on:** Ch. 1 (the cast and worlds named; the workspace cloned; $\mathcal{W}$, $R(q)$,
  $\mathcal{QO}_i$ first seen). Rust basics (Appendix A). The sister book's Chs. 3–4 for the ported
  $SE(2)$ type and simulator — the reader need not have read them; the port is documented here as an API.
- **Feeds into:** every chapter. Specifically Ch. 3 (`RangeSensor` is the Bug sensor; `World` is the
  arena), Ch. 4 (opens the `collide` box; adds Reach's Jacobian beside the FK defined here), Ch. 5
  (`trait Manifold` is implemented *for* the configuration types defined here), Chs. 7–14 and 17–21
  (every planner takes a `&dyn Collision` and a robot from `robots`), Ch. 23, Appendix F (reference).
- **Baseline sources:** NEW chapter mirroring the sister book's Ch. 4
  (`https://prvr.shannon.id/chapters/ch04-rusty-and-sensors`),
  whose `World`, `Rusty`, LiDAR, and determinism discipline are ported. C §3.8 (b. 69–71) for the
  forward kinematics map $\varphi : \mathcal{Q} \to M$ and Example 3.8.1 (the 2R arm) — **FK only**;
  the Jacobian and singularities of the same section are Ch. 4's. C App. F.5 (b. 509–512) as the black
  box (GJK, [C ref. 163] Gilbert, Johnson and Keerthi), named but not opened. C §3.1 (b. 40–42) for the
  circular robot's $R(q)$ and the 2R arm's $T^2$, quoted ahead of Ch. 4 because the types need them.
  Sister Ch. 3 (`…/chapters/ch03-geometry-of-motion`) for `Pose2` (SE(2)); sister Ch. 10
  (`…/chapters/ch10-sensor-models`) for the noisy LiDAR this chapter's planner-side sensor is *not*.
  Modernization: `parry2d` (Dimforge) documentation for the query vocabulary (verify 0.30 API).

## 3. Foundation (F) — Mathematical Core

Chapter notation table (symbols introduced here; book-wide ones are used without comment):

| Symbol | Meaning |
|---|---|
| $r$ | Rusty's body radius (0.11 m, inherited) |
| $\theta_1, \theta_2$ ($\theta_3$) | Reach's joint angles, each in $S^1$; $q = (\theta_1, \theta_2) \in T^2$ |
| $L_1, L_2$ ($L_3$) | Reach's link lengths; default $L_1 = L_2 = 1$ for every worked number |
| $\varphi : T^2 \to \mathbb{R}^2$ | Reach's forward kinematics (tip position) |
| $(x, y, \theta)$, $\psi$ | Hitch's body pose in $SE(2)$; trailer heading in $S^1$ |
| $\ell$, $\phi$, $d_h$ | Hitch's wheelbase, steering angle, hitch length |
| $\rho_R(x, s_k)$ | the range sensor's $k$-th ray, saturated at $R$ (full treatment Ch. 3) |
| $\operatorname{dist}(A, B)$, $(a^*, b^*)$ | distance between two shapes and its witness points |

**Definitions** (definition boxes):

- **Configuration type.** A value of a type that *is* a chart of $\mathcal{Q}$: `R2` for a translating
  disc, `T2` for Reach (angles wrapped to $(-\pi, \pi]$), `Pose2` for a planar rigid body,
  `HitchCfg = (Pose2, S1)` for car and trailer. Mixing them is a compile error, not a runtime check.
- **Footprint $R(q)$.** The compound shape the robot occupies at $q$: a disc of radius $r$ at $(x,y)$
  for Rusty (C §3.1: $R(x,y) = \{(x',y') : (x-x')^2 + (y-y')^2 \le r^2\}$); two (three) segments or
  thin capsules for Reach; a rectangle for Hitch's body and one for the trailer.
- **Forward kinematics** (C §3.8). A map $\varphi : \mathcal{Q} \to M$ between configuration
  representations — here joint angles to tip position. Not a homeomorphism even when dimensions match;
  for Reach the inverse is two-valued (elbow-up/down).
- **Collision query contract.** For a footprint $F$ and scene $S$: *intersects* ($F \cap S \ne
  \emptyset$), *distance* ($\min_{a \in F, b \in S}\|a-b\|$, zero when intersecting), *witness* (a
  minimizing pair $(a^*, b^*)$), *swept* (does $F$ moving rigidly from $q_a$ to $q_b$ along a straight
  chart line intersect $S$). These four, and nothing else, are what a planner may ask.
- **Range sensor (planner-side).** $\rho_R(x, s_k)$ for $n$ equally spaced bearings, by exact ray
  casting with no noise — *not* the sister's noisy LiDAR: Choset's Bug guarantees assume an ideal sensor.
- **Reproducible run.** $(\text{seed}, \text{input script}) \mapsto \text{trajectory}$ is a pure
  function; each stochastic subsystem owns one `SmallRng` stream split from the seed.

**Derivations / theorems:**

1. **Reach's forward kinematics** (C Example 3.8.1). *Statement:*
   $\varphi(\theta_1,\theta_2) = \big(L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2),\;
   L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2)\big)$. *Sketch (4 steps):* (i) the elbow is at
   $L_1(\cos\theta_1, \sin\theta_1)$; (ii) link 2 leaves it at absolute angle $\theta_1 + \theta_2$
   because joint angles are relative; (iii) add $L_2$ along that direction; (iv) the 3R variant appends
   $L_3(\cos(\theta_1+\theta_2+\theta_3), \sin(\cdot))$ and the tip heading (C problem 21).
   *Collapsible:* the same map as the $SE(2)$ product $T_{01}(\theta_1)T_{12}(\theta_2)T_{2,\text{tip}}$ — how `robots::Reach` computes it.
2. **Reach's workspace is an annulus, not a C-space** (C §3.1, b. 41). *Statement:* reachable tip
   positions are $\{x : |L_1 - L_2| \le \|x\| \le L_1 + L_2\}$; interior points are reached by exactly
   two configurations. *Sketch (3 steps):* triangle inequality on the two links; the law of cosines
   gives $\cos\theta_2$ from $\|x\|$, two signs; hence the tip position does not determine all body
   points. *Collapsible:* closed-form IK (C problem 20) as `Reach::ik -> [Option<T2>; 2]`.
3. **Collision of a disc is a point query.** *Statement:* Rusty at $(x,y)$ collides iff $D(x,y) < r$,
   $D$ the distance from the center to the nearest wall; the distance query is $D - r$. *Sketch (3
   steps):* a disc meets a set iff some set point is within $r$ of the center; that is $D < r$; the
   witness is the same point, shifted by $r$ along the normal. *Collapsible:* this is the Minkowski
   inflation of C §3.2.1 in one line, and why `collide::disc_distance` uses exact segment distance
   rather than GJK; the general case waits for Ch. 4.
4. **Hitch's kinematic model is a definition, not yet a theorem.** *Statement:* the simulator integrates
   $\dot x = v\cos\theta$, $\dot y = v\sin\theta$, $\dot\theta = (v/\ell)\tan\phi$, and for the trailer
   $\dot\psi = (v/d_h)\sin(\theta - \psi)$, exact-arc per tick. *Sketch (3 steps):* rear-axle no-slip
   gives the first two; the front wheel sets turning radius $\ell/\tan\phi$; the hitch drags the trailer
   axle. *Collapsible:* these are Pfaffian constraints whose consequences are Ch. 20's.

**Named algorithms** (book API names):

| Name | Signature | Complexity |
|---|---|---|
| `Reach::fk` | `(&self, q: T2) -> [Point2; 3]` (base, elbow, tip) | $O(1)$ |
| `Hitch::step` | `(&self, q: HitchCfg, u: CarCmd, dt) -> HitchCfg` | $O(1)$ |
| `Collision::intersects` | `(&self, fp: &Footprint) -> bool` | $O(k \log n)$, $k$ footprint parts, $n$ obstacles |
| `Collision::distance` / `::witness` | `(&self, fp) -> f64` / `-> Option<Witness>` | $O(k \log n)$ + GJK iterations (black box) |
| `Collision::swept` | `(&self, fp_a, fp_b) -> bool` | shape-cast, $O(k \log n)$ |
| `RangeSensor::scan` | `(&self, scene, x: Point2) -> Vec<f64>` | $O(n_{rays} \log n_{seg})$ |
| `Run::record` / `Run::replay` | `(cfg, script) -> Log` / `(&Log) -> Vec<Frame>` | $O(\text{ticks})$ |

**Numeric micro-example** (the contract with the tests in §5). Reach with $L_1 = L_2 = 1$ at
$q = (\pi/4, \pi/2)$ — Choset's own numbers from Example 3.8.1. Elbow $(\cos 45°, \sin 45°) =
(0.7071, 0.7071)$. Tip: $\theta_1 + \theta_2 = 135°$, so $\varphi(q) = (0.7071 + \cos 135°,\ 0.7071 +
\sin 135°) = (\mathbf{0},\ \sqrt{2}) \approx (0, 1.4142)$. Now a Workbench obstacle: the square
$[0.5, 1.0] \times [1.0, 1.5]$. Link 2 lies on the line $x + y = \sqrt 2 = 1.4142$; every point of the
square has $x + y \ge 1.5$, so the square is entirely on one side and
$\operatorname{dist} = (1.5 - \sqrt2)/\sqrt2 = 1.5/\sqrt2 - 1 \approx \mathbf{0.0607}$, with witnesses
$b^* = (0.5, 1.0)$ (the square's corner) and $a^* = (0.5, 1.0) - 0.0607\,(0.7071, 0.7071) \approx
(0.4571, 0.9571)$ on link 2 (between elbow and tip). `intersects` is false. For Rusty: center $(2.0, 1.0)$,
one wall from $(3,0)$ to $(3,3)$, $r = 0.11$: point distance $1.0$, so `distance` is $\mathbf{0.89}$, witness $(3,1)$.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor: *the flight simulator you get to open up — three cockpits, one engine.* Colors:
robot and executed path orange, workspace obstacles slate; a collision flash outlines the orange body
in goal red for one frame (the only time red means "bad," and the caption says so). Every widget
autoplays a seeded default, foregrounds one parameter, and ships a static SVG captured from the same code.

- **Widget w2.1: Three-Robot Dashboard** — type: wasm-sim (flagship, the lab's front door). Three
  panes: Rusty in the Apartment (arrow keys: $v, \omega$), Reach on the Workbench (drag either joint
  handle; the tip leaves an orange trail), Hitch in the Lot (arrow keys: $v$, steering $\phi$; the
  trailer follows). A collision flash and counter per pane. **Manipulates:** headline — the *active
  robot* selector (the others autoplay scripted tours); disclosure: Reach's link lengths, Hitch's
  trailer on/off and hitch length; seed re-roll (randomizes the tours, not the worlds). **Observes:**
  Reach's tip traces circles when one joint moves and the annulus boundary when extended; Hitch cannot
  move sideways no matter what the reader does; Rusty collides at distance $r$ from the wall, not zero.
  **Misconception killed:** *"driving the arm's tip is like driving the rover."* You drive the angles.
- **Widget w2.2: Query Anatomy** — type: interactive animation. A frozen Workbench scene with Reach at
  an adjustable $q$. Four tabs: *intersects* (the footprint fills red or stays orange), *distance* (a
  dashed segment between nearest points, length printed), *witness* ($a^*, b^*$ labeled; dragging $q$
  shows them jump between features), *swept* (a second configuration ghosted; the swept capsule drawn;
  a collision strictly between two free endpoints highlighted — why planners cannot check endpoints
  only). **Manipulates:** headline — drag $\theta_2$; disclosure: pick Rusty or Hitch, swept-step length.
  **Observes:** distance hits zero exactly where the fill turns red; the witness pair jumps at vertices.
  **Misconception killed:** *"a collision checker returns a boolean."* It returns geometry (Chs. 7–8 run on it).
- **Widget w2.3: World Atlas** — type: wasm-sim (light). The Apartment, Workbench, and Lot side by side
  with the labels the prose uses for twenty-one chapters (Apartment: the rooms and the long corridor,
  inherited; Workbench: the "block," the "post," the "shelf"; Lot: bays 1–4 and two pillars). Click to
  teleport the resident robot; hover shows coordinates and $R(q)$'s bounding box. One control: the
  world selector. Static fallback doubles as the book's map figure; the annotated chrome-contract figure
  (autoplay, speed, visible seed and re-roll, legend, `<noscript>` fallback) is inherited from the sister
  book as Appendix F's cover. **Misconception killed:** *"a world is a bitmap."* It is polylines and
  polygons with names, and the Ch. 7 grid is built *from* it.

Dashboard layout sketch (w2.1, the pattern every later dashboard copies): three aspect-locked canvases
in a row (Apartment widest); a control column at right with — top to bottom — playback chrome, the
active-robot selector, that robot's controls, the advanced disclosure, live readouts (the configuration
printed *as its type*: `T2 { th1: 0.785, th2: 1.571 }`), and the seed row; on narrow layouts the
canvases stack.

## 5. Practical (P) — Rust Implementation

**This section is the book's architectural contract.** Later chapters add crates and modules but
never restructure what is fixed here.

**Crates:** `nalgebra` 0.35 (`Point2`, `Isometry2`); `parry2d` 0.30 as `parry2d-f64` (shapes, `Bvh`,
`query::{distance, closest_points, intersection_test, cast_shapes}`; WASM-clean, no `rayon`); `rand`
0.9 with `SmallRng` (seeded streams; `Pcg64` where a long period matters); `serde` 1 for `Log`. Geometry
is the one thing this book does not hand-roll — Ch. 4 explains it, Appendix C documents it.

**Workspace layout (the contract):**

```text
crates/
  prob/          # PORTED: Rng = seeded SmallRng wrapper
  geom/          # PORTED: Pose2 = SE(2)
  edt/           # PORTED: exact Euclidean distance transform
  sim/           # PORTED: World (Apartment), Rusty (diff-drive step, encoders), ray cast
  robots/        # THIS CHAPTER: Reach<N> (FK), Hitch (car + trailer), Workbench, Lot scenes
  collide/       # THIS CHAPTER: trait Collision over parry2d; Footprint; Scene; RangeSensor
  widget-kit/    # THIS CHAPTER: Role enum (the only place a color is named), capture, replay harness
  ch01_hello/    # Ch. 1 (std-only)
  bugs/ cspace/ manifold/ search/ potential/ roadmap/ cells/ sampling/ multi/ estimate/
  dynamics/ trajectory/ trajopt/ nonholonomic/ learn/ capstone/      # added by their chapters
web/             # forked from the sister book; lib/{prob,geom,sim,mapping/edt} ported unchanged;
                 # lib/robots, lib/collide added here; later chapters add their own lib/ dirs
```

**Module plan:** `crates/robots/src/{reach.rs, hitch.rs, workbench.rs, lot.rs}`;
`crates/collide/src/{footprint.rs, scene.rs, collision.rs, sensor.rs}`; `crates/widget-kit/src/{role.rs,
capture.rs, replay.rs}`; `crates/sim/` ported file-for-file; `web/lib/robots/{reach.ts, hitch.ts,
worlds.ts}` and `web/lib/collide/{footprint.ts, scene.ts, sensor.ts}` as mirrors, pinned in `__checks__.ts`.

**Key types & signatures** (the API later chapters program against):

```rust
// crates/prob, crates/geom — PORTED. Names are the sister book's; do not redesign.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2 { pub x: f64, pub y: f64, pub theta: f64 }        // SE(2); compose/inverse/exp/log
pub struct Rng(rand::rngs::SmallRng);                             // seeded; split() for substreams
pub mod edt { pub fn exact_distance_field(/* rasterized walls */) -> DistanceField; }

// crates/sim — PORTED. World is the Apartment's polyline walls + Bvh; Rusty is the rover.
pub struct World { pub walls: Vec<Segment>, pub bounds: Aabb, /* bvh */ }
impl World { pub fn apartment() -> Self; pub fn raycast(&self, o: Point2<f64>, angle: f64, max: f64) -> Hit; }
pub struct Rusty { pub pose: Pose2, pub params: RobotParams /* body_radius 0.11 m */ }

// crates/robots/src/reach.rs — THIS CHAPTER: Reach and its FK (C §3.8, FK only).
/// Configuration types are charts of Q. Mixing them is a type error (Ch. 5 makes this a theorem).
#[derive(Clone, Copy, Debug, PartialEq)] pub struct R2(pub Point2<f64>);
#[derive(Clone, Copy, Debug, PartialEq)] pub struct T2 { pub th1: f64, pub th2: f64 }   // wrapped to (-π, π]
#[derive(Clone, Copy, Debug, PartialEq)] pub struct Tn<const N: usize>(pub [f64; N]);
#[derive(Clone, Copy, Debug, PartialEq)] pub struct HitchCfg { pub body: Pose2, pub trailer: Option<f64> }

/// Planar revolute arm with N links; Reach<2> is the book's Reach, Reach<3> the redundant variant.
pub struct Reach<const N: usize> { pub base: Point2<f64>, pub lengths: [f64; N], pub link_radius: f64 }
impl<const N: usize> Reach<N> {
    /// Joint positions base..tip — φ as a chain of SE(2) products (N+1 points via a helper const).
    pub fn fk(&self, q: &Tn<N>) -> Vec<Point2<f64>>;
    pub fn tip(&self, q: &Tn<N>) -> Point2<f64>;
    /// R(q): one capsule per link (link_radius = 0 gives exact segments, used by every worked number).
    pub fn footprint(&self, q: &Tn<N>) -> Footprint;
}
impl Reach<2> {
    /// Elbow-down, elbow-up; None when ‖x‖ is outside the annulus (C problem 20).
    pub fn ik(&self, tip: Point2<f64>) -> [Option<T2>; 2];
}

// crates/robots/src/hitch.rs
pub struct Hitch { pub wheelbase: f64, pub max_steer: f64, pub body: (f64, f64), pub hitch_len: Option<f64> }
pub struct CarCmd { pub v: f64, pub phi: f64 }
impl Hitch {
    /// Kinematic car (+ trailer) integrated exactly over dt; the constraint is *in* the model.
    pub fn step(&self, q: &HitchCfg, u: &CarCmd, dt: f64) -> HitchCfg;
    pub fn footprint(&self, q: &HitchCfg) -> Footprint;
}

// crates/collide — R(q) as data; the three worlds speak one language; THE black box.
pub struct Footprint { pub parts: Vec<(Isometry2<f64>, parry2d::shape::SharedShape)> }
pub struct Scene { obstacles: Vec<Obstacle /* name, SharedShape, Isometry2 */>, bvh: Bvh, pub bounds: Aabb }
impl Scene {
    pub fn from_world(w: &World) -> Self;     // Apartment walls become thin segment obstacles
    pub fn workbench() -> Self;               // table with the block, the post, the shelf
    pub fn lot() -> Self;                     // bays 1–4, two pillars, perimeter curb
}
pub struct Witness { pub on_robot: Point2<f64>, pub on_obstacle: Point2<f64>, pub dist: f64 }
pub trait Collision {
    fn intersects(&self, fp: &Footprint) -> bool;
    /// min over footprint parts and obstacles; 0.0 when intersecting (parry2d GJK — App. F.5)
    fn distance(&self, fp: &Footprint) -> f64;
    fn witness(&self, fp: &Footprint) -> Option<Witness>;
    fn swept(&self, fp_a: &Footprint, fp_b: &Footprint) -> bool;   // rigid straight-line sweep; shape-cast
}
impl Collision for Scene { /* the only module that imports parry2d::query */ }

/// The Bug sensor (C §2.2's ρ_R, implemented here; theory in Ch. 3). Exact, no noise.
pub struct RangeSensor { pub n_rays: usize /*360*/, pub max_range: f64 /*R*/ }
impl RangeSensor { pub fn scan(&self, scene: &Scene, x: Point2<f64>) -> Vec<f64>; } // INFINITY beyond R

// crates/widget-kit — the color code as a type; the determinism harness.
pub enum Role { Start, Goal, Obstacle, CObstacle, Graph, Path, Robot }   // --rm-start … --rm-robot
pub struct Log { pub seed: u64, pub frames: Vec<Frame> }  // serde; replay() must reproduce bit-for-bit
```

The deliberate compile error printed in the chapter: `reach.fk(&HitchCfg { .. })` — "expected `&Tn<2>`,
found `&HitchCfg`" — with the sentence "the compiler just told you $T^2 \ne SE(2) \times S^1$; Ch. 5 tells you why."

**Worked end-to-end example** (`cargo run -p robots --example lab_tour`): builds the three scenes,
places Reach at $(\pi/4, \pi/2)$, prints `elbow (0.7071, 0.7071)  tip (0.0000, 1.4142)`, adds the
square $[0.5,1.0]\times[1.0,1.5]$ to a scratch scene and prints `intersects: false  distance: 0.0607
witness a*=(0.4571, 0.9571) b*=(0.5000, 1.0000)`; then Rusty at $(2,1)$ against the wall
$(3,0)\!-\!(3,3)$: `distance: 0.8900`. Then it runs the 60 s three-robot tour at seed `BOOK_SEED`,
prints all three final configurations, and writes `figures/ch02_tour.svg`. Tests:
`fk_matches_choset_example` (tip to $10^{-12}$); `reach_distance_to_square` ($1.5/\sqrt2 - 1$ to
$10^{-9}$, witnesses to $10^{-6}$); `rusty_wall_distance` ($0.89$); `ik_roundtrip` (`fk(ik(fk(q)))
== fk(q)` for 1,000 seeded $q$); `swept_catches_midpoint` (two free Reach configurations whose sweep
crosses the post → `swept == true`); `tour_replays_bitwise` (every `Frame` equal exactly);
`scan_is_rho_R` (min over a 360-ray scan equals `Scene::distance` of a point to $10^{-6}$ — C problem 2.1).

**Runnable artifact:** the example above; in-page w2.1–w2.3 run `web/lib/robots` and `web/lib/collide`,
whose `__checks__.ts` entries assert the same tip, 0.0607, and 0.89. CI also builds the Rust crates for
`wasm32-unknown-unknown` as proof that nothing in the stack is WASM-hostile. The integration lab is the
tour: run it natively, run it in the browser, compare the final configurations digit for digit.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w2.1 | Three-Robot Dashboard | wasm-sim | `sim` + `robots` + `collide` + `widget-kit`; `web/lib/{sim,robots,collide}`; seeded `Rng` | drive Rusty, drag Reach's joints, steer Hitch; active-robot selector; seed re-roll | a configuration is a few typed numbers; $R(q)$ follows; Hitch cannot slide |
| w2.2 | Query Anatomy | interactive animation | `collide` + `robots`; `web/lib/collide` | drag $\theta_2$; query tabs; robot picker; swept step | what a checker returns: boolean, distance, witnesses, swept volume |
| w2.3 | World Atlas | wasm-sim (light) | `sim` + `robots` scenes | world selector; click-to-teleport; hover | the three worlds as named places; the chrome contract |
| — | Three-robot tour hook | animation (autoplay) | `robots` + `collide` replay | none | collision flashes; the lab exists |

## 7. Exercises & Extensions

1. **(F)** Derive Reach's inverse kinematics for general $L_1, L_2$ using the law of cosines and
   `atan2` (C problem 20); state the reachable annulus and show exactly which tip positions have one
   solution rather than two.
2. **(F)** Write $R(q)$ as a set for Hitch with the trailer: show it is the union of two rectangles
   whose poses are functions of $(x, y, \theta, \psi)$, and explain why $\psi$ is a configuration
   variable but the steering angle $\phi$ is not (a control).
3. **(C — predict, then verify with w2.2)** Put Reach at $(0, \pi/2)$ and center the square obstacle at
   $(1, 1)$ with side 0.2. Predict the distance and both witness points by hand, then check the distance
   tab. Slide $\theta_2$ toward $\pi/2 + 0.3$ and predict where the obstacle witness jumps corner → edge.
4. **(C — w2.1)** With the trailer on, try to make Hitch's trailer end up exactly one meter to the
   left of its start with the same heading. Describe the maneuver and count the direction reversals —
   Ch. 20 will tell you why zero is impossible.
5. **(P)** Add `Reach<3>` to w2.1 with `lengths = [0.6, 0.5, 0.4]` and show the tip can stay fixed
   while the arm moves (redundancy, C §3.3). Test: 200 seeded configurations with the same tip (via a
   1-D null-space walk) keep `tip` constant to $10^{-9}$.
6. **(P, stretch)** Replace the exact `RangeSensor` by the sister book's noisy `Lidar` port in the
   Bug1 stub from Ch. 1 and measure (100 seeds) how often the leave point is misjudged. One paragraph
   on why Ch. 3's guarantee needs the ideal sensor, and what a real robot does instead.

## 8. Modernization Notes

- **Kept from Choset.** The forward-kinematics map $\varphi$ and Example 3.8.1 with its exact numbers
  ($L_1 = L_2 = 1$, $q = (\pi/4, \pi/2)$), because they return in Ch. 4's Jacobian and Ch. 5's
  differential. §3.1's circular robot and 2R arm as the two canonical C-spaces. App. F.5's GJK as the
  *name* of what the black box does, with the note that modern engines add EPA, BVH broad phases, and
  shape-casting that Choset's appendix does not cover.
- **Added beyond 2005.** (i) Configurations as *types* with compile-time separation — Choset says "we
  use $q$"; we say `T2`, `Pose2`, `HitchCfg`. (ii) A production collision library (`parry2d`, verify
  API) behind a four-query contract, mirroring how OMPL/MoveIt treat FCL (Pan, Chitta & Manocha 2012,
  verify). (iii) Determinism as an engineering requirement with bit-exact replay tests, inherited from
  the sister book. (iv) The swept query, unnamed in Choset but behind every Part III edge check.
- **Dropped/condensed.** No re-derivation of the differential-drive model, odometry, or LiDAR noise —
  the sister book's Ch. 4 owns them. Hitch's kinematics are *stated* as the simulator's physics; Ch. 20
  derives the constraint, Ch. 21 the steering. `parry3d` is named here and used once, in Ch. 9.
- **Honesty items the prose must keep.** The planner-side `RangeSensor` is noiseless by design and the
  text says real LiDAR is not (sister Ch. 10). Zero-thickness links make Reach's worked numbers exact
  but unrealistic; the default Workbench uses `link_radius = 0.03` and the text shows both. `parry2d`
  distances carry floating tolerance, so tests compare to $10^{-9}$ — except the bit-exact determinism replay.
