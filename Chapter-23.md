# Chapter 23 — Capstone: A Planning Stack from Sensor to Motion

> Part VI — Frontiers and Integration · Estimated length: 12 web pages · Difficulty: Advanced

## 1. Purpose & Story Arc

Twenty-two chapters built planners; this chapter builds *the stack* — three of them, running at
once. Rusty navigates an Apartment it has never seen, replanning on a Part IV belief as doors turn
out to be shut; Reach plans a collision-free joint path, time-scales it to the torque limits, and
smooths it, on the Workbench; Hitch backs a trailer into a bay in the Lot without jackknifing. The
"aha" is architectural: a planning stack is a chain of *contracts* between stages, each stage a
different cell of Choset's task × robot × algorithm taxonomy (C Table 1.1), and the composed system
has exactly the guarantee of its weakest stage — no more, and only that much if every downstream
stage either verifies its input or falls back to it. The chapter
closes by mapping each stage onto its production counterpart in OMPL / MoveIt / Nav2 "(verify)",
with a retrospective on what Rust's type system caught while this book was written and what it
cost, and with the claim the book has been building toward: *everything you studied is running
here, unmodified, at real time, in the browser.*

Story line:

1. **Hook:** w23.1 runs on load — 60 seconds, three panels. Then the narrator closes a door in front
   of Rusty, drops Reach's goal at full stretch, and lets Hitch reverse until the trailer folds. All
   three recover; the chapter promises to explain every panel and every recovery.
2. **Problem and intuition (C):** why not one planner? Choset's taxonomy gives our three robots
   three algorithm columns and says what each cannot promise; boxes and arrows with rates show a
   plan computed for a world that has moved on.
3. **Formalize (F):** interface contracts (`Path<M>` vs `Trajectory<M>`, `Stamped<T>`), the
   guarantee-composition lemma, the replanning budget, validity horizons, the two guard statistics.
4. **Implementation (P):** the `Stage` trait, three pipelines, the supervisor's exhaustive `Mode`,
   guards and recoveries; native threads vs WASM round-robin with identical semantics.
5. **Experiment:** the failure tour (blocked corridor / singular arm / jackknife) run live; the
   library map; the retrospective; where to go next.

## 2. Prerequisites & Position

This is the integration chapter: it **builds on every chapter**. Exhaustive dependency table
(artifact = linked into `crates/capstone`; background = load-bearing concept, no code import):

| Chapter | Contribution to the capstone | Kind |
|---|---|---|
| Ch. 1 | task × robot × algorithm taxonomy (Table 1.1); path vs trajectory; completeness kinds — the frame the capstone closes | background |
| Ch. 2 | `sim`, `robots`, `collide`, `widget-kit`: the three worlds, Rusty/Reach/Hitch, Reach FK, range sensor, `Collision` trait | artifact |
| Ch. 3 | Tangent Bug as Rusty's local fallback when the grid plan is invalid; $D(x)$, $\nabla D$ from range data | artifact |
| Ch. 4 | `cspace`: disc inflation for Rusty, Reach's C-obstacle checks; `jacobian()` and $\sin\theta_2 = 0$ singularities — the arm guard | artifact |
| Ch. 5 | `trait Manifold` ($\mathbb{R}^2$, $T^2$, $SE(2)$, $SE(2)\times S^1$); the type every stage is generic over | artifact |
| Ch. 6 | `search`: D\* Lite replanner on Rusty's grid; A\* inside hybrid A\* | artifact |
| Ch. 7 | brushfire distance transform: clearance costs for Rusty, CHOMP's distance field for Reach | artifact |
| Ch. 8 | roadmap properties (accessibility/connectivity/departability) as the vocabulary of stage contracts | background |
| Ch. 9 | why Rusty does not trace the GVG online: grid replanning is cheaper at Apartment scale | background |
| Ch. 10 | coverage — the one Table 1.1 task the capstone does not run; pointer box | background |
| Ch. 11 | `sampling`: PRM, kd-tree, `Steer`; Reach's `Collision`-backed edge checks | artifact |
| Ch. 12 | RRT-Connect: Reach's first feasible solution | artifact |
| Ch. 13 | RRT\* / Informed RRT\*: anytime refinement within the planning budget | artifact |
| Ch. 14 | composite manifolds (Hitch + trailer as $SE(2)\times S^1$); forward-propagation preview | background |
| Ch. 15 | EKF odometry belief and NIS gate: Rusty's pose covariance for inflation margins | artifact |
| Ch. 16 | particle localizer + `replan_on_belief`: the loop Rusty's panel runs | artifact |
| Ch. 17 | `dynamics`: Reach's $M(q), C(q,\dot q), g(q)$ and torque limits | artifact |
| Ch. 18 | `trajectory::time_scale`: path → time-optimal trajectory | artifact |
| Ch. 19 | `trajopt`: CHOMP smoothing for Reach; MPC path tracker for Rusty | artifact |
| Ch. 20 | `nonholonomic`: Pfaffian constraint check; trailer kinematics; the hitch-angle guard | artifact |
| Ch. 21 | hybrid A\* with Reeds–Shepp shots; flatness-based docking steer for the trailer | artifact |
| Ch. 22 | optional learned sampler for Reach's PRM (toggle); benchmark data for the retrospective | artifact |
| App. A–F | Rust idioms, topology card, collision reference, completeness definitions, linear systems card, simulator framework | background |

- **Feeds into:** nothing — terminal chapter. Appendix F documents the framework w23.1 stresses.
- **Baseline sources:** whole book (TOC §3); Choset read for framing, owned elsewhere — C Ch. 1
  (§1.1–1.3, owned by Ch. 1; b. 1–15): Table 1.1 and the taxonomy prose (b. 9–11);
  the path/trajectory distinction (b. 14–15); RHINO's "interleaving sensing, planning, and action"
  (b. 4), which names Rusty's panel. C §7.5 (owned by Chs. 14/21; b. 253–266): control-based planning (§7.5.1, Hitch's
  forward-propagation lineage via LaValle and Kuffner [C ref. 272]) and the §7.5.2–7.5.6 list the
  capstone leaves to Ch. 14. Choset has **no systems chapter**; the systems framing follows OMPL
  (Şucan, Moll & Kavraki "(verify)"), MoveIt "(verify)", and Nav2 "(verify)" architecture
  descriptions. Sister book Ch. 26 (bus/task pattern, chaos buttons); sister Chs. 12–13, 19, 23.

## 3. Foundation (F) — Mathematical Core

**Notation introduced this chapter:**

| Symbol | Meaning |
|---|---|
| $\mathcal{S}_i$, $\tau_i$, $\varsigma_i$ | stage $i$; its period; its worst-case staleness ($= \tau_i$ for a periodic producer) |
| $T_{react}$ | worst-case sense-to-command delay through a pipeline |
| $d_{detect}$, $v_{max}$, $a_{max}$ | distance at which a new obstacle is first trusted; Rusty's speed and braking limits |
| $L_{valid}$ | validity horizon: path prefix re-checked against the newest map before execution |
| $\mu(q) = \lvert\det J(q)\rvert = l_1 l_2 \lvert\sin\theta_2\rvert$ | Reach's manipulability (2R, square Jacobian) |
| $\phi$, $\phi_{max}$, $\phi_{warn}$ | hitch angle $\theta_{trailer} - \theta_{car}$; hard bound; guard threshold |
| $\mathcal{G}$ | a guarantee: complete ≻ resolution-complete ≻ probabilistically complete ≻ none |

**Definitions:**

- **D23.1 Stage and contract.** A stage consumes `Stamped` inputs and emits one `Stamped` output
  at period $\tau_i$; its contract names input type, output type, frame, and the guarantee
  $\mathcal{G}_i$ it provides *given* valid inputs. Types are the contract: `Path<M>` is a geometric
  curve in $\mathcal{Q}_{free}$, `Trajectory<M>` is time-parameterized (C §1.3); nothing executes a `Path`.
- **D23.2 Validity horizon.** $L_{valid} = v_{max}(\tau_{plan} + \varsigma_{map})$: the path prefix
  that may be driven before the next plan arrives, re-checked against the newest map every tick.
- **D23.3 Composed guarantee.** $\mathcal{G}(\text{pipeline}) = \min_i \mathcal{G}_i$ over stages
  that can *reject* a query, provided every post-processor verifies its output or falls back to
  its input; otherwise $\mathcal{G} = $ none.
- **D23.4 Mode.** The supervisor's discrete state per robot — an exhaustive enum (§5):
  `Plan | Execute | Replan | Recover(kind) | Done | Failed(reason)`.

**Key derivations** (name · statement · sketch · collapsible):

1. **Guarantee composition (weakest link, verified post-processors).** *Statement:* Rusty — D\*
   Lite on the inflated grid is resolution-complete, the MPC tracker is a verified post-processor,
   Tangent Bug is a complete fallback for the disc in the *true* world; the panel is
   resolution-complete *given* a localized belief and a static world, and the text states both
   conditions. Reach — RRT-Connect/RRT\* are probabilistically complete; time scaling succeeds on
   any collision-free path whose velocity-limit curve admits a phase-plane trajectory (Ch. 18);
   CHOMP may *break* collision-freeness, so it is wrapped verify-or-fall-back; composed:
   probabilistically complete. Hitch — hybrid A\* is resolution-complete w.r.t. its primitive set
   and the hitch-angle bound; Reeds–Shepp shots and flatness docking are exact; composed:
   resolution-complete w.r.t. the lattice. *Sketch (4 steps):* list stages and $\mathcal{G}_i$;
   identify rejecting stages; check each post-processor's fallback; take the minimum.
   *Collapsible:* the three tables; a Ch. 22 sampler at $\lambda > 0$ leaves Reach's row unchanged.
2. **Replanning budget (staleness chain + braking).** *Statement:* a newly blocked corridor first
   trusted at distance $d_{detect}$ is handled iff
   $$v_{max}\,T_{react} + \frac{v_{max}^2}{2a_{max}} < d_{detect},\qquad
   T_{react} = \varsigma_{scan} + \varsigma_{bel} + \varsigma_{map} + \tau_{plan} + \tau_{ctrl}.$$
   *Sketch (3 steps):* worst-case delay = one period per stage (message produced just after the
   consumer read); add braking distance; compare with the sensing horizon. *Collapsible:* the
   arithmetic with the defaults (the micro-example); the moving-obstacle variant adds $v_{obs}T_{react}$.
3. **Singularity guard (Ch. 4 Jacobian).** *Statement:* for a tip velocity $\dot x$, $\|\dot q\| \le
   \|\dot x\|/\sigma_{\min}(J)$, and for the 2R arm $\sigma_{\min} \to 0$ exactly when $\mu(q) = l_1
   l_2|\sin\theta_2| \to 0$; the guard rejects goals with $\mu(q_{goal}) < \mu_{min}$. *Sketch (4
   steps):* $\dot q = J^{-1}\dot x$; SVD bound; $\det J$ for 2R from Ch. 4; threshold. Recovery:
   joint-space time scaling (Ch. 18) is singularity-agnostic — its limits are torques, not Cartesian
   speeds — so the stack pulls the tip target inward by $\delta$ and replans in joint space.
   *Collapsible:* the 3R variant with $\mu = \sqrt{\det JJ^\top}$.
4. **Jackknife guard (Ch. 20 constraint).** *Statement:* with the trailer hitched at the rear axle,
   $\dot\phi = -\frac{v}{d}\sin\phi - \frac{v}{L}\tan\delta$; linearized about $\phi = 0$, $\dot\phi
   \approx -(v/d)\phi$ — stable forward, unstable in reverse with rate $|v|/d$. The guard fires at
   $|\phi| > \phi_{warn} < \phi_{max}$. *Sketch (4 steps):* trailer kinematics from Ch. 20/21;
   difference of heading rates; linearize; sign of $v$. Recovery: drive forward ($v > 0$ restores
   $\phi \to 0$ exponentially) until $|\phi| < \phi_{ok}$, then hybrid A\* replans with primitives
   filtered by $|\phi| \le \phi_{max}$. *Collapsible:* the Pfaffian form of the trailer constraint
   and why $\phi_{max}$ is a *state* constraint layered on it.

**Named algorithms** (signature · complexity / rate; defaults: Apartment grid $240 \times 180$ cells
at 5 cm, Reach on $T^2$, Lot lattice $(x, y, \theta)$ at $10\,\text{cm} \times 10\,\text{cm} \times 15°$):

| Algorithm | Signature | Complexity / rate |
|---|---|---|
| `pipeline_tick` | `(bus, stages, now) -> ()` — run due stages in topological order (WASM) | sum of below; 60 fps kept |
| `rusty_stack` | scan → Ch. 16 belief → Ch. 4 inflation (margin from Ch. 15 covariance) → Ch. 6 D\* Lite → D23.2 check → Ch. 19 MPC; Ch. 3 Tangent Bug fallback | D\* Lite incremental; MPC $O(H)$ at 20 Hz |
| `reach_stack` | Ch. 12 RRT-Connect → Ch. 13 RRT\* anytime (budget) → Ch. 18 `time_scale` → Ch. 19 CHOMP → verify (Ch. 11 checker) → execute | $O(n\log n)$; $O(N_s)$; $O(kN_s)$ |
| `hitch_stack` | Ch. 21 hybrid A\* (RS shots, $\lvert\phi\rvert \le \phi_{max}$ filter) → Ch. 21 flatness docking → tracker | lattice A\* $O(E\log V)$ |
| `supervisor_step` | `(Mode, Events) -> Mode` — D23.4, one machine per robot | $O(1)$ |
| `corridor_blocked` | `(path, grid) -> Option<Cell>` — first occupied cell within $L_{valid}$ | $O(L_{valid}/\Delta)$ |
| `singularity_guard` / `jackknife_guard` | `(q) -> Result<(), Singular{mu}>` (F3); `(phi, v) -> Result<(), Jackknife{phi}>` (F4) | $O(1)$ |
| `replanning_budget` | `(RateTable, v_max, a_max, d_detect) -> BudgetReport` — F2 | $O(1)$ |
| `run_mission` | `(seed, MissionCfg) -> MissionReport` — all three pipelines, deterministic | full mission |

**Numeric micro-example** (the contract with the §5 test): the F2 inequality with the book's
default rates — range sensor 10 Hz ($\varsigma_{scan} = 0.10$ s), Ch. 16 belief 10 Hz ($0.10$),
inflated-grid update 5 Hz ($0.20$), D\* Lite replanner 2 Hz ($\tau_{plan} = 0.50$), MPC tracker
20 Hz ($\tau_{ctrl} = 0.05$); Rusty $v_{max} = 0.6$ m/s, $a_{max} = 1.0$ m/s²; $d_{detect} = 2.0$ m
(two consecutive scans must confirm a cell; sensor range 4 m).
$$T_{react} = 0.10 + 0.10 + 0.20 + 0.50 + 0.05 = 0.95\ \text{s};\quad
0.6 \times 0.95 + \frac{0.6^2}{2 \times 1.0} = 0.570 + 0.180 = \mathbf{0.750\ \text{m}} < 2.0\ \text{m}.$$
Margin $1.25$ m. The maximum safe speed solves $v^2/2 + 0.95v - 2.0 = 0$:
$v_{safe} = -0.95 + \sqrt{0.9025 + 4.0} = \mathbf{1.264\ \text{m/s}}$ (check: $1.264 \times 0.95 +
1.264^2/2 = 1.201 + 0.799 = 2.000$). Validity horizon $L_{valid} = 0.6 \times (0.50 + 0.20) = 0.42$ m.

## 4. Conceptual (C) — Intuition & Visual Design

Chapter metaphor in one line: *a stack is a relay race — each runner only promises to carry what
the previous one handed over, and the baton has a type.* Colors per TOC §2 — start **green**, goal
**red**, obstacle **slate**, C-obstacle **amber hatched**, roadmap/tree/explored **blue**, path
**purple**, robot and executed trajectory **orange**. Rusty's belief uses the sister's estimation colors unchanged; since posterior purple collides with
this book's path purple, the particle cloud lives in the **Belief** tab (ellipse outline only on the
canvas). Ground truth is a toggle, default off.

- **Widget w23.1: The Grand Plan** — wasm-sim, full-page (flagship, TOC name). Three world panels:
  **Apartment** (Rusty; grid inflated by Ch. 4 with a Ch. 15 covariance margin; D\* Lite path in
  purple; MPC rollout in orange; live scan), **Workbench** (Reach; unwrapped torus with amber
  C-obstacles; RRT\* tree in blue; time-scaled path in purple with the phase-plane $(s, \dot s)$
  strip beneath; CHOMP-smoothed version overlaid), **Lot** (Hitch and trailer; hybrid A\*
  expansions in blue; Reeds–Shepp cusps marked; hitch-angle gauge). Left rail: three mode machines
  (D23.4), the **chaos buttons** — *Close Door*, *Singular Goal*, *Reverse Hard* — seed, speed,
  ground-truth and Ch. 22 learned-sampler toggles. Right rail, tabbed inspectors: **Belief**,
  **Search**, **Tree**, **Phase Plane**, **Guards** ($\mu(q)$, $\phi$ against thresholds), **Timing**
  (staleness bars against the F2 budget).
  Bottom: shared timeline with event pins. **Manipulates:** headline — the chaos buttons;
  disclosure — seed, speed, toggles, tabs. **Observes:** three sense→plan→act loops at once; on
  sabotage, a guard statistic crossing its threshold, a mode switch, a recovery. **Misconception
  killed:** "an autonomy demo is one clever algorithm" — it is three chains of stated contracts,
  and every failure is a contract violation you can point at. Autoplay on load; static fallback: a six-frame SVG storyboard.
- **Widget w23.2: Stack Anatomy** — wasm-sim (trace replay). The §5 block diagram with live
  message-rate counters on each arrow and staleness heat on each block, replaying a recorded
  mission. Click a block: chapter badge, input/output types, period, $\mathcal{G}_i$. Toggle a block
  off and watch degradation ripple: D\* Lite off → Rusty drives a stale path until the D23.2 check
  stops it; time scaling off → the Reach tracker refuses a `Path` (the rustc message is shown);
  jackknife guard off → the Lot panel folds. **Manipulates:** headline — block on/off; disclosure —
  trace picker, speed. **Misconception killed:** architecture diagrams as decoration — here the
  arrows carry measured traffic and removing a box has a visible, causal consequence.
- **Widget w23.3: Failure Tour** — animation (three curated recorded missions, scrubbable). Tab
  *Blocked Corridor*: door closes → two scans confirm → grid updates → D\* Lite repairs only the
  affected cells → new purple path → MPC follows; a variant with the replanner throttled to 0.5 Hz
  shows the D23.2 check stopping Rusty short. Tab *Singular Arm*: goal at full stretch →
  $\mu(q_{goal})$ dives below $\mu_{min}$ → `Recover(PullGoalInward)` → joint-space replan → time
  scaling unaffected. Tab *Jackknife*: reverse with $\phi$ growing exponentially → $\phi_{warn}$
  crossed → `Recover(PullForward)` → $\phi \to 0$ → hybrid A\* replans with filtered primitives →
  parked. **Manipulates:** headline — tab; disclosure — time scrub. **Misconception killed:**
  recovery as magic — each is a statistic crossing a threshold plus a mode with a plan.
- **Widget w23.4: Retrospective Scorecard** — static-svg, two equal panels. Panel A: five real
  compile-error vignettes — a `Torus` path handed to Hitch's `Se2` tracker (Ch. 5 `Manifold`
  mismatch); a `Path` passed where a `Trajectory` is required (D23.1); a planner `q` in the `map`
  frame fed to the `odom`-frame belief (`FrameId` newtype); a non-exhaustive `match` when
  `Recover(Jackknife)` was added; a grid mutated by the replanner while the tracker held a borrow —
  each as the actual rustc message with a one-line moral. Panel B, the costs: generic-heavy compile times, WASM single-threading, `candle`'s WASM
  weight (why Ch. 22 has `infer.rs`), kd-tree crates without metric hooks (hand-rolled in Ch. 11).
  **Misconception killed:** "Rust is free safety" — both panels, same size.

Dashboard layout sketch (w23.1): `[left rail 220px: 3 mode machines / chaos / seed·speed] [center:
three world panels in a row, Workbench with phase-plane strip] [right rail 300px: tab strip +
inspector] / [bottom 140px: shared timeline + event pins]`.

## 5. Practical (P) — Rust Implementation

**Crates** (inherited through the integrated workspace crates; the capstone adds one): `nalgebra`
0.35, `parry2d` 0.30, `petgraph` 0.8, `pathfinding` 4.15, `rand` 0.9 (`SmallRng`, seeded) — via the
Chs. 2–22 crates; `crossbeam-channel` 0.5 — bounded MPMC channels for the native bus (the one new dependency).
**Concurrency design (the chapter's key engineering content):** stages implement one trait; on
native each runs on its own thread at its own rate over channels; on WASM (no threads) a
deterministic round-robin scheduler ticks due stages inside the frame loop — *same stage code, same
message types, same seeds, bit-identical missions.*

**Module plan:** `crates/capstone/`

```
src/
  bus.rs          // Stamped<T>, FrameId, Msg, Bus (native: channels; wasm: queues)
  stage.rs        // Stage trait (D23.1), ThreadRunner (native), RoundRobin (wasm)
  stacks/rusty.rs // Ch. 16 belief → Ch. 4 inflation → Ch. 6 D* Lite → D23.2 → Ch. 19 MPC; Ch. 3 fallback
  stacks/reach.rs // Ch. 12 → Ch. 13 → Ch. 18 → Ch. 19 → verify (Ch. 11 checker)
  stacks/hitch.rs // Ch. 21 hybrid A* (+ Ch. 20 constraint filter) → flatness docking → tracker
  guards.rs       // singularity_guard (F3), jackknife_guard (F4), corridor_blocked (D23.2)
  budget.rs       // replanning_budget (F2) + RateTable defaults
  supervisor.rs   // Mode machine per robot; recoveries
  mission.rs      // run_mission, MissionCfg, MissionReport
```

**Key types & signatures:**

```rust
use {manifold::Manifold, trajectory::{Path, Trajectory, time_scale}}; // Ch. 5; Ch. 18 owns both:
// `Path<M>` is geometric, `Trajectory<M>` is time-stamped (D23.1). Trackers accept only the
// latter; a `Path` cannot be executed — the compiler says so. The capstone adds no new types here.

pub struct FrameId(pub &'static str);   // "map", "odom", "base" — a planner q and a belief x_t
pub struct Stamped<T> { pub t: SimTime, pub frame: FrameId, pub v: T } // never meet without a frame

/// One pipeline stage; identical impls run threaded (native) or cooperatively (wasm).
pub trait Stage: Send {
    type In; type Out;
    fn period_hz(&self) -> f64;
    fn guarantee(&self) -> Guarantee;   // D23.3: what this stage promises given valid input
    fn tick(&mut self, input: &Stamped<Self::In>, now: SimTime) -> Option<Stamped<Self::Out>>;
}
pub enum Guarantee { None, ProbabilisticallyComplete, ResolutionComplete, Complete } // derives Ord: min = weakest
pub fn composed_guarantee(stages: &[&dyn StageInfo]) -> Guarantee;   // min over rejecting stages

pub enum Recover { PullGoalInward { delta: f64 }, PullForward { until_phi: f64 }, TangentBug }
pub enum Mode { Plan, Execute, Replan, Recover(Recover), Done, Failed(FailReason) } // exhaustive
pub fn singularity_guard(q: &robots::ReachConfig, mu_min: f64) -> Result<(), Singular>;
pub fn jackknife_guard(phi: f64, v: f64, phi_warn: f64) -> Result<(), Jackknife>;

pub fn replanning_budget(r: &RateTable, v_max: f64, a_max: f64, d_detect: f64) -> BudgetReport;

pub struct MissionCfg { pub seed: u64, pub rates: RateTable, pub chaos: Vec<ChaosEvent> }
pub struct MissionReport {
    pub rusty: PanelReport, pub reach: PanelReport, pub hitch: PanelReport, // per-panel metrics below
    pub events: Vec<(SimTime, StackEvent)>, pub budget: BudgetReport,
}
pub fn run_mission(cfg: &MissionCfg) -> MissionReport;
```

**Worked end-to-end example** (`cargo run --release --example mission -- --seed 7 --close-door-at
12 --singular-goal --reverse-hard`): prints the budget line first — `t_react 0.950 s  stop 0.750 m
margin 1.250 m  v_safe 1.264 m/s  l_valid 0.42 m` — then the event timeline (door closed → corridor blocked → replan; singular goal → pull
inward; jackknife → pull forward), then the report: Rusty goal reached, path 18.4 m vs 16.9 m
optimal on the final map (ratio 1.09), 3 replans; Reach cost 2.31 rad, duration 1.84 s → 1.71 s
after CHOMP, max $|\tau|/\tau_{max} = 0.97$; Hitch parked within 4 cm / 1.8°, 2 cusps, max
$|\phi| = 0.62$ rad (design targets; a test pins the implementer's actuals under the fixed seed). The no-chaos
seed-7 mission is the canonical regression test: CI fails if any panel metric regresses. Tests: `budget_matches_hand_computation` (0.750, 1.264, 0.42 to 1e-3),
`composed_guarantee_is_min`, `trajectory_required_to_execute` and `mode_match_exhaustive`
(compile-fail tests via `trybuild`), `mission_seed_7_pinned`.

**Runnable artifact:** `run_mission` natively; the WASM Grand Plan is the same machinery stepped by
the round-robin scheduler with every internal streamed to the inspectors. The `web/lib/` TypeScript
port mirrors the stack for the default page engine; the Rust build is CI-tested on
`wasm32-unknown-unknown` and selectable on the page, so the book's last sentence is literal.

## 6. Simulations & Animations Manifest

| id | title | type | crate stack | interaction | teaches |
|---|---|---|---|---|---|
| w23.1 | The Grand Plan | wasm-sim (full-page) | capstone + sim/robots/collide + cspace, search, sampling, estimate, dynamics, trajectory, trajopt, nonholonomic, learn (toggle) | chaos buttons; seed, speed, toggles, inspector tabs | three stacks of stated contracts; detection + recovery are mechanisms |
| w23.2 | Stack Anatomy | wasm-sim (trace replay) | capstone bus trace, widget-kit | block on/off; trace picker | architecture as measured dataflow; degradation is causal and typed |
| w23.3 | Failure Tour | animation (recorded, scrubbable) | recorded missions, widget-kit | tab per failure; scrub | F1's contract table, one violation per tab |
| f23.2 | fig-23-library-map | static-svg | hand-set | none | our stages ↔ OMPL / MoveIt / Nav2 "(verify)" |
| w23.4 | Retrospective Scorecard | static-svg | hand-set rustc snippets | none | what the type system caught vs what it cost |

## 7. Exercises & Extensions

1. **(F)** Extend F2 to a walker crossing at $v_{obs}$: derive the inequality, then compute the
   fastest walker the default rates tolerate at $d_{detect} = 2.0$ m and $v_{max} = 0.6$ m/s.
2. **(F)** Write all three pipelines as D23.3 chains and prove the composed guarantee of each from
   F1. Which single stage would you upgrade first, and what would it cost in $T_{react}$?
3. **(C, predict-then-verify)** Using the Timing tab, predict the replanner rate below which the
   *Blocked Corridor* tour stops Rusty short of the door at $v_{max} = 1.0$ m/s. Verify by
   throttling the planner in w23.1. Which term of F2 dominated?
4. **(P)** Add `Mode::ReturnHome` for Rusty: after `Done`, plan and drive back to $q_{start}$. Touch
   only `supervisor.rs` and `mission.rs`; the compiler must force you to handle the new mode in
   every `match` (w23.4's fourth vignette, firsthand).
5. **(P, stretch)** Replace Hitch's hybrid A\* with Ch. 21's kinodynamic RRT\* behind the same `Stage` contract
   and jackknife guard; re-run seed 7 and the *Jackknife* tour. Does `composed_guarantee` change?

## 8. Modernization Notes

- **Kept from Choset, and why:** Ch. 1's taxonomy (Table 1.1, b. 10) is the spine — the capstone
  is the first place the reader fills every cell for a concrete robot; the path/trajectory
  distinction (§1.3, b. 14–15) becomes a type; RHINO's "interleaving sensing, planning, and action"
  (b. 4) is Rusty's panel; §7.5.1's control-based planning (b. 253–254) is Hitch's lineage. Choset has
  **no integration chapter**; this chapter makes the systems lore explicit and executable.
- **Added beyond 2005 (each "(verify)"):** the architecture mirrors OMPL's planner/state-space
  separation (Şucan, Moll & Kavraki, IEEE RAM 2012), MoveIt's planning-scene → planner →
  time-parameterization → post-processing chain, and Nav2's costmap → planner server → controller
  server → behavior-tree recoveries; D\* Lite as the replanner (Koenig & Likhachev, from
  Ch. 6's modernization set). Ecosystem pointers (`rclrs`/`r2r` ROS 2 bindings, `rapier`, `parry`)
  are dated October 2026 and flagged as such — they rot fastest.
- **Deliberately reused rather than re-derived:** every algorithm is a citation into Chs. 2–22;
  the only new formal content is composition math (F1–F4). The capstone must prove the book's
  artifacts compose, not introduce a 24th planner. §7.5.2–7.5.6 (multi-robot, manipulation,
  assembly, flexible objects, biology) stay in Ch. 14's pointer box: three robots in three worlds is
  already the honest limit of one browser tab.
- **Honesty items the prose must keep:** (1) the three panels do not share a world — this is three
  stacks, not one robot team; the text says so and points to Ch. 14 for coupling. (2) The simulator
  grades its own homework: Rusty's path ratio against the *final-map optimum* and Hitch's park
  error against ground truth exist because we own the world; the retrospective says what evaluation
  looks like on hardware. (3) "Complete" claims are conditional — F1 states the conditions
  (localized belief, static world, primitive set) and the panel displays `composed_guarantee`
  rather than letting the reader infer more. (4) WASM is cooperative, not preemptive — the scheduler is deterministic, and the text must
  not imply the browser proves real-time *scheduling*, only *throughput*. (5) The retrospective reports costs with the same care as wins (w23.4 panels
  equal-sized; compile-time numbers measured, not vibed).
