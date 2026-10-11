# Chapter 1 — Robots That Plan: The Piano Mover's Problem

> Part I — Foundations: Robots, Worlds, and Configuration Space · Estimated length: 6 web pages · Difficulty: Foundational

## 1. Purpose & Story Arc

This chapter is the book's thesis statement and voice calibration. It has to convince an engineer who
already knows how to make a robot *move* that making it decide *where* to move is a discipline with
theorems, not a bag of heuristics — and that the discipline rests on one idea Choset et al. put in
their first paragraph and never let go of: a robot, however many joints it has, is a **point** in the
right space. The "aha" the reader leaves with: **motion planning does not happen in the room; it
happens in configuration space, where the piano is a point and the walls have become shapes the
point must avoid.** Everything else in the book — bugs, potentials, roadmaps, samplers, trajectories,
Lie brackets — is a way of finding a curve for that point. The voice set here governs all 23 chapters:
hook first, playful when it helps, and every playful claim backed immediately by a figure the reader
can drag, a number the reader can check, or a `cargo run` that prints that number.

Story line:

1. **Hook** — w1.1 autoplays: a point robot with a contact sensor crawls around two obstacles toward a
   goal it cannot see past, using nothing but "walk toward it, and if you bump, go around." It
   arrives. One sentence of narration: "This is a provably complete planner, and it fits in forty
   lines."
2. **Play (C)** — w1.2 Piano Mover: drag a sofa through a doorway; beside the room, a single dot
   moves in a space the reader has never seen, and the doorway has become a narrow channel between
   two amber blobs. The reader discovers that the sofa gets stuck exactly when the dot touches amber.
3. **Formalize (F)** — Choset's four motivating problem classes (piano mover, Mini AERCam, personal
   transport vehicles, coverage/demining), his three-axis taxonomy (task · robot · algorithm),
   path vs. trajectory, and the five symbols the whole book is written in: $\mathcal{W}$,
   $\mathcal{WO}_i$, $\mathcal{Q}$, $R(q)$, $\mathcal{QO}_i$.
4. **Algorithm** — Bug1 stated informally (motion-to-goal, boundary-following, leave at the closest
   point), with its path-length guarantee quoted and checked by hand on a text world.
5. **Implementation (P)** — `ch01_hello`: a dependency-free Rust binary that runs Bug1 on a 9 × 5
   text world and prints the path and its length; the unit test pins the number.
6. **Integration lab** — the reader clones the workspace, builds in seconds, reproduces the number,
   then edits the obstacle and watches the bound stay true. Meet the cast: Rusty, Reach, Hitch, and
   their three worlds, all built in Ch. 2.
7. **Close** — the FCP contract, the color code, and a one-page map of the book: which chapters
   answer which cell of the taxonomy.

## 2. Prerequisites & Position

- **Builds on:** nothing — this is the entry point. Assumed: fluency in some programming language
  (Rust not required yet; Appendix A is the on-ramp), vectors and matrices at the "seen them before"
  level, and comfort with the idea of a function.
- **Feeds into:** Ch. 2 (the cast and worlds named here are built there; the workspace cloned here is
  the lab), Ch. 3 (Bug1 is derived and bounded properly; w1.1 grows into w3.1 Bug Arena), Ch. 4 (the
  amber blobs of w1.2 are computed exactly as Minkowski sums and $SE(2)$ slices), Ch. 6 and Appendix
  D (the completeness taxonomy stated here is formalized), Ch. 21 (the parallel-parking example
  becomes a theorem about Lie brackets), Ch. 23 (the capstone closes the loop the hook opens), every
  chapter (voice, notation, color code, FCP rhythm).
- **Baseline sources:** C Ch. 1 — the motivating examples (b. 1–9), §1.1 Overview of Concepts in
  Motion Planning (b. 9–11; Table 1.1), §1.2 Overview of the Book (b. 12–13), §1.3 Mathematical Style
  (b. 13–15; eq. 1.1 and figure 1.10). Sister-book cross-links: Ch. 1
  (`https://prvr.shannon.id/chapters/ch01-robot-that-doubts`) for
  the shared hook-first pedagogy and the shared protagonist, Ch. 20
  (`…/chapters/ch20-motion-planning`) as the sister's one-chapter preview that this whole book
  expands. Modernization set: OMPL (Şucan, Moll & Kavraki 2012) as the systems framing for "what a
  planner is today" (verify).

## 3. Foundation (F) — Mathematical Core

Deliberately light — the only chapter where C outweighs F — but nothing stated here is retracted
later. Chapter-scoped notation (first appearance of the book-wide symbols from the global table):

| Symbol | Meaning | Formalized in |
|---|---|---|
| $\mathcal{W} \subset \mathbb{R}^2$ or $\mathbb{R}^3$ | workspace, the ambient space the robot's body lives in | Ch. 2 |
| $\mathcal{WO}_i$, $\mathcal{W}_{free} = \mathcal{W} \setminus \bigcup_i \mathcal{WO}_i$ | the $i$-th workspace obstacle; free workspace | Ch. 2 |
| $\mathcal{Q}$, $q$ | configuration space; one configuration (a point the planner imagines) | Ch. 4–5 |
| $R(q) \subset \mathcal{W}$ | the set of workspace points occupied by the robot at $q$ | Ch. 2 |
| $\mathcal{QO}_i = \{q : R(q) \cap \mathcal{WO}_i \ne \emptyset\}$, $\mathcal{Q}_{free}$ | C-obstacle; free configuration space | Ch. 4 |
| $c : [0,1] \to \mathcal{Q}_{free}$ | a path; a trajectory when the parameter is time | Ch. 18 |
| $q_{start}$, $q_{goal}$ | the query | Ch. 3 |
| $d(x, y)$ | Euclidean distance in the plane (Bug algorithms); a metric in general | Ch. 5 |
| $p_i$ | perimeter of workspace obstacle $i$ (Bug bounds) | Ch. 3 |

**Definitions** (named, one crisp sentence each, as definition boxes):

- **Configuration.** A complete specification of the position of every point of the robot; the
  configuration space $\mathcal{Q}$ is the set of all of them, and $\dim \mathcal{Q}$ is the number
  of degrees of freedom (C b. 2: the piano has six).
- **Path-planning problem** (C eq. 1.1). Find a continuous $c : [0,1] \to \mathcal{Q}$ with
  $c(0) = q_{start}$, $c(1) = q_{goal}$, and $c(s) \in \mathcal{Q}_{free}$ for all $s$; the choice of
  $[0,1]$ is a convention, any parameterization serves.
- **Path vs. trajectory.** A path is a curve ($C^0$); a trajectory is a path parameterized by time
  and must be at least $C^2$ so velocities and accelerations exist (C b. 14–15). Finding one is
  *trajectory planning*; Choset also calls it motion planning, and this book keeps his distinction.
- **Offline vs. online, sensor-based.** Offline planners finish before execution from a known model;
  online planners build the plan while moving; sensor-based planners interleave sensing, computation,
  and action (C b. 11). The boundary is "murky" and is one of computation time — Choset's remark
  that a fast enough offline planner in a feedback loop *is* online is kept, because it anticipates
  MPC (Ch. 19).
- **Omnidirectional vs. nonholonomic; kinematic vs. dynamic.** Free to move instantaneously in any
  direction of $\mathcal{Q}$ vs. subject to velocity constraints (C b. 4, 10) — the sofa vs. Hitch;
  velocities as controls (Parts I–IV) vs. forces as controls (Part V).
- **Completeness, three kinds** (C b. 11). *Complete:* finds a solution whenever one exists or reports
  failure in finite time. *Resolution complete:* finds it if one exists at the chosen discretization.
  *Probabilistically complete:* the probability of finding one, if it exists, tends to 1 as time
  grows. Optimality, completeness, and computational complexity trade off; the book states which of
  the three each planner has, every time.

**Derivations / theorems** (statement, sketch, collapsible):

1. **The robot is a point (the C-space reduction).** *Statement:* for any robot and any workspace
   obstacles, $c$ is a collision-free motion of the body iff $c$ is a curve in $\mathcal{Q}_{free}$ —
   planning for a body is planning for a point in a different space. *Sketch (4 steps):* (i) $R(q)$
   is a function of $q$ alone, by the definition of configuration; (ii) the robot at $q$ collides with
   $\mathcal{WO}_i$ iff $q \in \mathcal{QO}_i$, by definition of $\mathcal{QO}_i$; (iii) hence a curve
   avoids all collisions iff it avoids $\bigcup_i \mathcal{QO}_i$; (iv) $\mathcal{Q}_{free}$ is the
   complement, so (1.1) is exactly the collision-free condition. *Collapsible:* why the reduction is
   not free — the C-obstacles are usually not computable in closed form (Ch. 4 does the cases that
   are), $\mathcal{Q}$ is usually not $\mathbb{R}^n$ (Ch. 5), and the dimension is the enemy
   (Appendix D).
2. **Dimension counting for the piano.** *Statement:* a free rigid body in $\mathbb{R}^3$ has six
   degrees of freedom; in the plane, three. *Sketch (3 steps):* position of one body point (3);
   orientation (3: roll–pitch–yaw, or any chart of $SO(3)$); rigidity fixes every other point.
   *Collapsible:* Ch. 4's point-counting argument with holonomic constraints (C §3.3), and the
   warning that six *coordinates* do not make $\mathcal{Q}$ equal to $\mathbb{R}^6$ (Ch. 5).
3. **Bug1 is complete and its path is bounded** (quoted, not yet proved). *Statement* (C eq. 2.1):
   for a point robot with a contact sensor in a bounded planar workspace,
   $L_{Bug1} \le d(q_{start}, q_{goal}) + 1.5 \sum_i p_i$, and Bug1 reports failure in finite time when
   no path exists. *Sketch (3 steps):* each obstacle met costs at most one full circumnavigation plus
   half a perimeter to reach the best leave point; straight-line segments total at most $d$; finitely
   many obstacles. *Collapsible:* a pointer — the proof, the failure test, and Bug2's weaker bound are
   Ch. 3's business; here the inequality is *checked*, numerically, on the text world below.
4. **Why "just drive toward the goal" is not a planner.** *Statement:* greedy descent on
   $d(\cdot, q_{goal})$ gets stuck in any workspace with a concave obstacle facing the start.
   *Sketch (3 steps):* construct a U-shaped obstacle around the goal direction; at the bottom of the
   U every direction increases $d$; the controller is a local minimum of a potential, which Ch. 7
   names. *Collapsible:* this is Choset's warning (C b. 12) that "purely reactive gradient-following
   approaches always run the risk of getting stuck," and the book's first instance of the local-minimum
   disease.

**Named algorithms** (introduced informally; the formal statements are C Alg. 1–3 in Ch. 3):

| Name | Signature | Complexity |
|---|---|---|
| `bug1` (C Alg. 1, informal) | `(world, q_start, q_goal) -> Result<Path, Unreachable>` | path length $\le d + 1.5\sum p_i$; each boundary step is $O(1)$ sensing |
| `motion_to_goal` | `(x, q_goal, contact) -> Step` | $O(1)$ per step |
| `follow_boundary` | `(x, contact, hand: Left/Right) -> Step` | $O(1)$ per step (wall-following on the text grid) |

**Numeric micro-example** (the chapter's contract with the unit test in §5). A 9-column × 5-row text
world, origin at the top-left, 4-connected unit moves, a point robot that occupies one cell and whose
contact sensor reports whether the next cell is `#`:

```text
.........
...###...
S..###..G
...###...
.........
```

$q_{start} = (0,2)$, $q_{goal} = (8,2)$, one 3 × 3 obstacle occupying columns 3–5, rows 1–3. Its
perimeter, measured as the ring of 16 free cells the robot walks when circumnavigating, is $p_1 = 16$.
Bug1 by hand: (i) motion-to-goal along row 2: $(1,2), (2,2)$ — 2 steps; the next cell $(3,2)$ is `#`,
so $q^H_1 = (2,2)$. (ii) Circumnavigate the ring back to $(2,2)$: 16 steps, recording $d(\cdot, q_{goal})$
at each cell; the unique minimum is $(6,2)$ at distance 2. (iii) Walk to the leave point $q^L_1 = (6,2)$
by the shorter way round: 8 steps either way (the ring is symmetric; the code breaks the tie clockwise).
(iv) Motion-to-goal $(7,2), (8,2)$: 2 steps. Total $L_{Bug1} = 2 + 16 + 8 + 2 = \mathbf{28}$ steps.
Bound: $d + 1.5\,p_1 = 8 + 24 = 32 \ge 28$. For contrast (Ch. 3 will explain): Bug2 on the same world
leaves the ring the first time it meets row 2 closer to the goal, at $(6,2)$ after 8 steps, for
$2 + 8 + 2 = 12$, which here is also the shortest 4-connected path. The printed map marks the path
with `·`, the hit point `H`, the leave point `L`.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor, carried end-to-end: **the piano is a point.** Book color roles from the first
figure on: start **green**, goal **red**, workspace obstacles **slate**, C-obstacles **amber hatched**,
the robot and its executed path **orange**, the planned path **purple**; roadmaps/trees (**blue**) do
not appear until Ch. 6. Every widget autoplays a seeded default, exposes one headline parameter, and
degrades to a static SVG rendered from the same code.

- **Widget w1.1: Bug Preview** — type: wasm-sim (autoplay). A point robot (orange) with a contact
  sensor runs Bug1 around two slate obstacles between a green start and a red goal; the m-line is
  drawn faintly; hit and leave points are labeled $q^H_i$, $q^L_i$ as in C figure 2.1. A length
  counter and the bound $d + 1.5\sum p_i$ sit beneath. **Manipulates:** one headline control — drag
  the goal; behind a disclosure: switch to Bug2, obstacle preset (Choset's figure 2.1 vs. the
  "comb" that makes Bug2 lose); seed re-roll for the obstacle layout. **Observes:** the robot never
  gets stuck; the counter never exceeds the bound; moving the goal behind the obstacle lengthens the
  circumnavigation but not the guarantee. **Misconception killed:** *"a planner without a map is a
  heuristic."* It is a theorem with a contact sensor.
- **Widget w1.2: Piano Mover** — type: wasm-sim (drag). Left pane: a room with a doorway and a sofa
  (a rectangle) the reader drags and rotates with a handle. Right pane: the sofa's configuration
  $(x, y, \theta)$ shown as the $\theta$-slice of $SE(2)$ at the current heading — a point moving
  among amber hatched C-obstacles that redraw as $\theta$ changes (computed by the Ch. 4 star
  algorithm, used here as a black box). **Manipulates:** headline — the sofa's length; disclosure:
  doorway width, show all $\theta$-slices stacked as a translucent 3D block (C figure F.8).
  **Observes:** the sofa jams exactly when the point touches amber; a sofa that fits the doorway only
  diagonally shows a channel that exists at some $\theta$ and closes at others — the first sighting
  of a narrow passage. **Misconception killed:** *"the obstacle is the wall."* The obstacle the
  planner fights is the amber shape, and it depends on the robot.
- **Widget w1.3: The Planner's Taxonomy** — type: static-svg (hover reveals). Choset's Table 1.1 as a
  three-column board — task (navigate / cover / localize / map), robot (C-space and DOF;
  omnidirectional vs. nonholonomic; kinematic vs. dynamic), algorithm (optimal vs. not; complexity;
  the three completeness kinds; online/offline; sensor-based vs. world model). Hovering a cell
  lights the chapters that occupy it (e.g., "sensor-based + navigate + complete" → Chs. 3, 9).
  **Misconception killed:** *"motion planning means finding the shortest path."* Most of the board
  is not about optimality at all.
- **Widget w1.4: How to Read This Book** — type: static-svg. The FCP rhythm (Hook → C → F → P → Lab →
  Exercises) and the seven-role color legend as a keyed strip, reused in the site header and
  Appendix F. Establishes that equation terms (`\htmlClass{term-start}{…}` and friends), figure
  elements, and code comments (`// --rm-goal`) share colors.

Dashboard layout sketch: w1.1 sits above the fold before any prose, full width, counter and bound
docked under the canvas. w1.2 is a two-pane card (workspace left ≥ 55 %, C-space slice right) placed
at the "the piano is a point" paragraph; the two panes share one cursor so the reader sees the dot
move while dragging the sofa. w1.3 is inline with the taxonomy section; w1.4 closes the chapter
beside the setup instructions. On narrow layouts the two panes of w1.2 stack vertically.

## 5. Practical (P) — Rust Implementation

**Crates:** none — `ch01_hello` is std-only, edition 2024, by design: the reader's first build must
finish in seconds and cannot fail on a native dependency. The workspace `Cargo.toml` already pins the
book stack from the TOC (`nalgebra` 0.35, `parry2d` 0.30 as `parry2d-f64`, `rand` 0.9, `petgraph` 0.8,
`pathfinding` 4.15), but nothing in this chapter touches it. The in-page widgets w1.1/w1.2 run the
`web/lib/` ports of `bugs` (Ch. 3) and `cspace` (Ch. 4) through the inherited `SimCanvas`/`WidgetFrame`
framework; Ch. 1 is authored against artifacts the reader has not yet seen built, as the sister's is.

**Module plan:** `crates/ch01_hello/` (one `[[bin]]`, `src/main.rs`, ~80 lines printed in full;
`tests/worked_example.rs`). Reader setup path (stated step by step, CI-tested on Linux/macOS/Windows):
install `rustup` → stable toolchain (edition 2024 needs ≥ 1.85) → `git clone <book repo> && cd
robot-motion-rust` → `cargo run -p ch01_hello` → `cargo test -p ch01_hello`. Optional, deferred to
Ch. 2: `rustup target add wasm32-unknown-unknown` for the WASM cross-check build; Node ≥ 20.9 for the
`web/` site.

**Key types & signatures** (the whole program, compiles in spirit):

```rust
// crates/ch01_hello/src/main.rs — zero dependencies, edition 2024
/// A text world: '#' is obstacle, everything else is free. 4-connected unit moves.
struct TextWorld { rows: Vec<Vec<u8>> }
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Cell { x: i32, y: i32 }

impl TextWorld {
    fn parse(s: &str) -> (Self, Cell, Cell);          // finds 'S' and 'G'
    /// The contact sensor: is the cell blocked (or outside the bounded workspace)?
    fn blocked(&self, c: Cell) -> bool;
}

/// Euclidean distance d(x, y) — Bug1 may measure it, by assumption (C §2.1).
fn dist(a: Cell, b: Cell) -> f64;

/// One step of motion-to-goal: the 4-neighbor that most reduces d(., goal).
/// Returns None when that neighbor is blocked — the hit point.
fn motion_to_goal(w: &TextWorld, x: Cell, goal: Cell) -> Option<Cell>;

/// One step of boundary following with the obstacle kept on the robot's right hand
/// (the grid analogue of the surface-normal rule of C §2.3.1). O(1).
fn follow_boundary(w: &TextWorld, x: Cell, heading: Dir) -> (Cell, Dir);

enum Outcome { Reached(Vec<Cell>), Unreachable { at: Cell } }

/// Bug1, C Algorithm 1: motion-to-goal; on contact circumnavigate fully, remembering the
/// perimeter point closest to the goal; return to it the shorter way; repeat. If leaving the
/// leave point toward the goal would re-enter the obstacle, the goal is unreachable.
fn bug1(w: &TextWorld, start: Cell, goal: Cell) -> Outcome;

fn render(w: &TextWorld, path: &[Cell], hit: &[Cell], leave: &[Cell]) -> String;

fn main() {
    let (w, s, g) = TextWorld::parse(WORLD);
    match bug1(&w, s, g) {
        Outcome::Reached(path) => {
            let steps = path.len() - 1;
            let perimeter = 16;                                 // counted by the run, printed too
            println!("{}", render(&w, &path, &[], &[]));
            println!("steps: {steps}  straight-line: {}  bound d + 1.5·Σp: {}",
                     dist(s, g), dist(s, g) + 1.5 * perimeter as f64);
        }
        Outcome::Unreachable { at } => println!("goal unreachable; detected at {at:?}"),
    }
}
```

**Worked end-to-end example:** `cargo run -p ch01_hello` prints the map with the 28-step path marked,
`H` at $(2,2)$, `L` at $(6,2)$, then `steps: 28  straight-line: 8  bound d + 1.5·Σp: 32`. The test
`worked_example_ch01` asserts `steps == 28`, `perimeter == 16`, and `steps as f64 <= 8.0 + 1.5 * 16.0`;
a second test `unreachable_is_detected` encloses the goal in `#` and asserts `Outcome::Unreachable`
within one circumnavigation — the first instance of the book-wide rule that every printed number is
locked by a test. A third test, `bug2_text_world`, is a stub that Ch. 3 fills in (expected 12).

**Runnable artifact:** the binary above. The in-page w1.1/w1.2 run the `web/lib/plan/bugs.ts` and
`web/lib/cspace/` ports (owned by Chs. 3–4); `lib/__checks__.ts` asserts the same 28 on the same text
world, so the animation the reader watched and the number they printed are one computation. The
integration lab is the reader's own machine: edit `WORLD` to a 4 × 4 block, predict the new length
($2 + 20 + 10 + 2 = 34$ against a bound of $8 + 30 = 38$), run, check.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w1.1 | Bug Preview | wasm-sim (autoplay) | `bugs` (Ch. 3) via `web/lib/plan/bugs.ts`; seeded `Rng` | drag goal; Bug1/Bug2 toggle; preset; seed re-roll | a contact sensor and two behaviors give a complete planner with a length bound |
| w1.2 | Piano Mover | wasm-sim (drag) | `cspace` (Ch. 4) star algorithm via `web/lib/cspace/`; `geom` `Pose2` | drag/rotate sofa; sofa length; doorway width; stacked slices | the planner's obstacle is the C-obstacle; the robot is a point |
| w1.3 | The Planner's Taxonomy | static-svg (hover) | build-time SVG + CSS hover | hover cells → chapter map | Choset's three-axis taxonomy; where each chapter lives |
| w1.4 | How to Read This Book | static-svg | build-time SVG | none | FCP rhythm; the seven color roles |
| — | Terminal run (figure) | static (code output) | `ch01_hello` | none | the 28-step text-world path, as printed |

## 7. Exercises & Extensions

1. **(F)** Classify each of Choset's ten motivating examples (piano mover, Mini AERCam, CyCab/Segway,
   RHINO, Sojourner, demining, painting arm, snake robots, surgical robots, digital actors, protein
   folding) on all three taxonomy axes. Which cells of Table 1.1 are empty? Name one 2026 robot that
   fills each empty cell.
2. **(F)** The sofa in w1.2 has $\dim \mathcal{Q} = 3$. Count the degrees of freedom of (a) two sofas
   in the same room, (b) two sofas tied by a rope of fixed length pulled taut, (c) a sofa on casters
   that cannot roll sideways. State for each whether the constraint changes $\dim \mathcal{Q}$ and
   why (holonomic vs. nonholonomic — anticipating C §3.3).
3. **(C — predict, then verify with w1.1)** Set the goal directly behind the larger obstacle. Before
   pressing play: which side will Bug1 leave from, and will the counter land closer to $d$ or to
   the bound? Verify; then switch to Bug2 and explain in one sentence why its counter differs.
4. **(C — w1.2)** Shorten the sofa until it fits the doorway straight on. Predict how the amber
   slice changes as you rotate the sofa by 90°, then verify. Which $\theta$ gives the widest channel,
   and why is that not $\theta = 0$ when the doorway is in a side wall?
5. **(P)** Modify `ch01_hello` so the obstacle is a `C` shape opening toward the start. Predict the
   path length and the bound before running. Then make the contact sensor *lie* with probability
   0.05 (use a fixed seed with a tiny hand-rolled LCG — still zero dependencies) and report what
   breaks: the length, the guarantee, or both.
6. **(P, stretch)** Implement Bug2 in `ch01_hello` by changing only `follow_boundary`'s exit
   condition (leave when back on the fixed m-line, closer to the goal, and able to step toward it).
   Make `bug2_text_world` pass with 12. Then construct a text world where Bug2's count exceeds
   Bug1's — you have just drawn C figure 2.4.

## 8. Modernization Notes

- **Kept from Choset, and why.** The motivating examples (b. 1–9) are kept as a *gallery*, not a
  survey: the piano mover, the Mini AERCam (the one example that forces "trajectory" rather than
  "path"), the CyCab/Segway parallel-parking problem (the first nonholonomic sighting), and demining
  (the first time completeness is the whole point) each carry a definition. Table 1.1's taxonomy is
  kept verbatim because it still partitions the field and because the book's chapter map is built on
  it. §1.3's notation and eq. (1.1) are adopted unchanged; this book's global table is Choset's
  Appendix A extended.
- **Added beyond 2005.** (i) The Bug1 hook made executable and numeric on day one — Choset's Ch. 1
  has no worked number; our 28-step text world is the book's first "numeric example = unit test."
  (ii) The Piano Mover widget shows $SE(2)$ C-obstacle slices live; in 2005 this was figure F.8, a
  static picture. (iii) The taxonomy board links each cell to the modern systems that occupy it —
  OMPL/MoveIt/Nav2 (verify, from the modernization set) — so the reader sees that "planner" is now
  infrastructure, not a research artifact. (iv) The cast (Rusty, Reach, Hitch) replaces RHINO,
  Sojourner, and the ABB painter as the in-book protagonists; the historical robots stay in the
  gallery.
- **Dropped/condensed.** The surgery, digital-actor, and drug-design examples become one paragraph
  with pointers (Ch. 14's biology pointer box picks up protein folding); §1.2's chapter-by-chapter
  overview is replaced by w1.3's board and the TOC; the museum-tour-guide localization story is
  handed to the sister book, whose Ch. 1 tells it properly.
- **Honesty items the prose must keep.** Bug1's guarantee requires perfect positioning, a bounded
  workspace, and a point robot — the text says so before the counter starts. The text-world Bug1 is
  a 4-connected caricature; Ch. 3 does the continuous version. "Complete" is a property of an
  algorithm *under stated assumptions*, never of a demo.
