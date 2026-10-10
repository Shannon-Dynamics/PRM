/**
 * The capstone's bookkeeping as data: what every chapter contributes to the
 * running stack, how each stage maps onto the production libraries, and the
 * retrospective's two equal columns. Widgets draw it; the prose cites it; the
 * check file asserts the "artifact" rows are true of this module's imports.
 */

export interface DependencyRow {
  chapter: string;
  slug: string | null;
  contribution: string;
  /** artifact = imported and executed by `lib/capstone`; background = load-bearing concept, no import. */
  kind: 'artifact' | 'background';
  /** The `lib/` modules the browser port imports for this row (empty for background). */
  imports: readonly string[];
}

export const DEPENDENCIES: readonly DependencyRow[] = [
  { chapter: 'Ch. 1', slug: 'ch01-robots-that-plan', contribution: 'Task × robot × algorithm taxonomy (Choset Table 1.1); path vs trajectory; the completeness vocabulary the composed guarantee is written in', kind: 'background', imports: [] },
  { chapter: 'Ch. 2', slug: 'ch02-the-lab', contribution: 'The three robots and worlds; the Collision contract (DiscChecker, ChainChecker, FootprintChecker) that every dense recheck calls; the range sensor', kind: 'artifact', imports: ['robots', 'collide', 'sim/world'] },
  { chapter: 'Ch. 3', slug: 'ch03-bug-algorithms', contribution: 'Tangent Bug as Rusty’s complete local fallback when the inflated grid has no path', kind: 'artifact', imports: ['bugs'] },
  { chapter: 'Ch. 4', slug: 'ch04-configuration-space-i', contribution: 'Disc inflation (Rusty’s C-obstacles on the grid); Reach’s Jacobian, |det J| and σ_min — the singularity guard', kind: 'artifact', imports: ['cspace/jacobian'] },
  { chapter: 'Ch. 5', slug: 'ch05-configuration-space-ii', contribution: 'The manifolds every stage is generic over: T² for Reach’s planner and shortcut, ℝ² for Rusty’s', kind: 'artifact', imports: ['manifold'] },
  { chapter: 'Ch. 6', slug: 'ch06-graph-search', contribution: 'D* Lite on Rusty’s inflated grid; the grid raster of the blueprint; A* for the final-map optimum', kind: 'artifact', imports: ['search'] },
  { chapter: 'Ch. 7', slug: 'ch07-potential-functions', contribution: 'The brushfire distance transform under CHOMP’s distance field for Reach', kind: 'artifact', imports: ['potential (via trajopt/field)'] },
  { chapter: 'Ch. 8', slug: 'ch08-roadmaps-i', contribution: 'Accessibility, connectivity, departability — the vocabulary of a stage contract', kind: 'background', imports: [] },
  { chapter: 'Ch. 9', slug: 'ch09-roadmaps-ii', contribution: 'Why Rusty does not trace the GVG online: at Apartment scale a repaired grid plan is cheaper', kind: 'background', imports: [] },
  { chapter: 'Ch. 10', slug: 'ch10-cell-decompositions', contribution: 'Coverage — the one Table 1.1 task the capstone does not run', kind: 'background', imports: [] },
  { chapter: 'Ch. 11', slug: 'ch11-prm', contribution: 'The Steer contract and greedy shortcutting; Reach’s FreeSpace over the chain checker', kind: 'artifact', imports: ['sampling (steer, free-space)'] },
  { chapter: 'Ch. 12', slug: 'ch12-tree-planners', contribution: 'RRT-Connect: Reach’s first feasible path and the source of its probabilistic completeness', kind: 'artifact', imports: ['sampling (tree)'] },
  { chapter: 'Ch. 13', slug: 'ch13-optimal-sampling', contribution: 'Informed RRT*, anytime inside an iteration budget', kind: 'artifact', imports: ['sampling (optimal)'] },
  { chapter: 'Ch. 14', slug: 'ch14-many-bodies', contribution: 'Composite manifolds and coupling — why three robots in three worlds are three stacks, not a team', kind: 'background', imports: [] },
  { chapter: 'Ch. 15', slug: 'ch15-kalman-observer', contribution: 'The 3σ margin and the covariance vocabulary of the belief gate (the browser port reads σ from Chapter 16’s cloud)', kind: 'background', imports: [] },
  { chapter: 'Ch. 16', slug: 'ch16-bayesian-localization', contribution: 'The particle localizer Rusty plans from; replan_on_belief scoring the first move on D* Lite’s field', kind: 'artifact', imports: ['estimate'] },
  { chapter: 'Ch. 17', slug: 'ch17-robot-dynamics', contribution: 'Reach’s M(q), C(q, q̇), g(q) and torque limits', kind: 'artifact', imports: ['dynamics'] },
  { chapter: 'Ch. 18', slug: 'ch18-trajectory-planning', contribution: 'Path-constrained dynamics, the time-optimal Time-Scaling Algorithm, and the Trajectory<M> type', kind: 'artifact', imports: ['trajectory'] },
  { chapter: 'Ch. 19', slug: 'ch19-trajectory-optimization', contribution: 'CHOMP smoothing for Reach; the receding-horizon iLQR tracker for Rusty', kind: 'artifact', imports: ['trajopt'] },
  { chapter: 'Ch. 20', slug: 'ch20-nonholonomic-i', contribution: 'The car-with-trailer drive field behind the hitch-angle rate of the jackknife guard', kind: 'artifact', imports: ['nonholonomic (systems)'] },
  { chapter: 'Ch. 21', slug: 'ch21-nonholonomic-ii', contribution: 'Hybrid A* with Reeds–Shepp shots; the trailer simulated along a car path and the path scorer', kind: 'artifact', imports: ['nonholonomic (lattice, car-path)'] },
  { chapter: 'Ch. 22', slug: 'ch22-learning-and-planning', contribution: 'The optional learned sampler for Reach’s RRT-Connect, shipped only inside a uniform floor', kind: 'artifact', imports: ['learn'] },
  { chapter: 'App. A–F', slug: null, contribution: 'Rust idioms, topology card, collision reference, completeness definitions, linear-systems card, simulator framework', kind: 'background', imports: [] },
];

/**
 * Our stages and their production counterparts. Component names are as the
 * projects' documentation used them in October 2026 — the fastest-rotting
 * column in this book; check the current docs before relying on one.
 */
export interface LibraryRow {
  stage: string;
  chapter: string;
  ompl: string;
  moveit: string;
  nav2: string;
}

export const LIBRARY_MAP: readonly LibraryRow[] = [
  { stage: 'Configuration space', chapter: 'Ch. 5', ompl: 'StateSpace (SE2, RealVector, compound)', moveit: 'robot model joint groups', nav2: '2-D costmap only' },
  { stage: 'Collision contract', chapter: 'Ch. 2', ompl: 'StateValidityChecker + MotionValidator', moveit: 'PlanningScene collision checks', nav2: 'costmap footprint checks' },
  { stage: 'Inflated map', chapter: 'Chs. 4, 6', ompl: '—', moveit: 'planning scene world geometry', nav2: 'costmap_2d static, obstacle, inflation layers' },
  { stage: 'Belief', chapter: 'Ch. 16', ompl: '—', moveit: '—', nav2: 'AMCL' },
  { stage: 'Grid replanner', chapter: 'Ch. 6', ompl: '—', moveit: '—', nav2: 'planner server (NavFn, Smac 2D, Theta*)' },
  { stage: 'Sampling planners', chapter: 'Chs. 12–13', ompl: 'RRTConnect, RRTstar, InformedRRTstar', moveit: 'OMPL planner plugin', nav2: '—' },
  { stage: 'Shortcut', chapter: 'Ch. 11', ompl: 'PathSimplifier', moveit: 'OMPL path simplification', nav2: 'smoother server' },
  { stage: 'Time scaling', chapter: 'Ch. 18', ompl: '—', moveit: 'time-parameterization adapter (time-optimal)', nav2: '— (controllers emit velocities)' },
  { stage: 'Trajectory optimization', chapter: 'Ch. 19', ompl: '—', moveit: 'CHOMP and STOMP planners', nav2: 'MPPI controller' },
  { stage: 'Tracker', chapter: 'Ch. 19', ompl: '—', moveit: 'trajectory execution + ros2_control', nav2: 'controller server (DWB, Regulated Pure Pursuit, MPPI)' },
  { stage: 'Car planning', chapter: 'Ch. 21', ompl: 'ReedsSheppStateSpace, DubinsStateSpace, control planners', moveit: '—', nav2: 'Smac Hybrid-A* and State Lattice' },
  { stage: 'Guards and recoveries', chapter: 'Chs. 4, 20', ompl: 'PlannerTerminationCondition (budgets)', moveit: 'Servo singularity thresholds', nav2: 'behavior server (Spin, BackUp, Wait)' },
  { stage: 'Supervisor', chapter: 'D23.4', ompl: '—', moveit: '—', nav2: 'BT Navigator (behavior trees)' },
];

export interface Vignette {
  title: string;
  code: string;
  error: string;
  moral: string;
}

/**
 * Panel A of the Retrospective Scorecard: five mistakes the type system
 * refused. The diagnostics are rustc's for the listings as printed in this
 * chapter, abridged (paths and line numbers elided); the error codes are rustc's.
 */
export const RETRO_WINS: readonly Vignette[] = [
  {
    title: 'A torus path to a car tracker',
    code: 'hitch_tracker.follow(&reach_path);',
    error: 'error[E0308]: mismatched types\n  expected `&Path<Se2>`, found `&Path<Torus<2>>`',
    moral: 'Chapter 5’s Manifold parameter is the contract: an arm’s joint path cannot steer a car.',
  },
  {
    title: 'A Path where a Trajectory is required',
    code: 'reach_tracker.execute(shortcut_path);',
    error: 'error[E0308]: mismatched types\n  expected `Trajectory<Torus<2>>`, found `Path<Torus<2>>`',
    moral: 'D23.1 as a type: nothing executes a curve that has no clock. Chapter 18 is not optional.',
  },
  {
    title: 'A map-frame pose into the odom-frame belief',
    code: 'belief.correct_with(goal_in_map);',
    error: 'error[E0308]: mismatched types\n  expected `Stamped<Pose2, Odom>`, found `Stamped<Pose2, Map>`',
    moral: 'Frames as zero-sized marker types: the mix-up is a compile error, not a silent drift.',
  },
  {
    title: 'A new recovery, an old match',
    code: 'match mode { … }   // after adding Recover::PullForward',
    error: 'error[E0004]: non-exhaustive patterns:\n  `Mode::Recover(Recover::PullForward { .. })` not covered',
    moral: 'D23.4’s exhaustive Mode: every supervisor that forgot the jackknife stops compiling.',
  },
  {
    title: 'Replanning under a reader',
    code: 'let ahead = path_view(&grid); dstar.update_cells(&mut grid, changes);',
    error: 'error[E0502]: cannot borrow `grid` as mutable\n  because it is also borrowed as immutable',
    moral: 'The tracker’s view and the replanner’s write cannot overlap — the race D23.2 exists to catch at run time.',
  },
];

/** Panel B, the costs — reported with the same care, and without invented numbers. */
export const RETRO_COSTS: readonly { title: string; detail: string }[] = [
  {
    title: 'Generic-heavy compile times',
    detail: 'Every planner is generic over Manifold, Collision and Steer, and is monomorphized once per robot. That is what makes the five errors above possible, and it is paid at every build. This edition did not measure the cost, so it prints no number for it.',
  },
  {
    title: 'WASM is single-threaded',
    detail: 'Fifteen stages that would each own a thread share one round-robin inside a frame. The schedule is deterministic and the throughput is real; preemption and worst-case latency are not demonstrated by a browser at all.',
  },
  {
    title: 'candle is heavy in WASM',
    detail: 'Chapter 22 trains with candle but ships a forty-line infer.rs forward pass, so the learned sampler runs in the page without the framework.',
  },
  {
    title: 'kd-tree crates lack metric hooks',
    detail: 'Nearest neighbours on T² and SE(2) need periodic axes; Chapter 11 hand-rolls its kd-tree for exactly that reason and keeps the off-the-shelf crate as a flat-ℝᵈ cross-check in tests.',
  },
  {
    title: 'Two implementations to keep honest',
    detail: 'The Rust in the prose and the TypeScript in the page are kept in step by hand and by checks; the browser’s supervisor matches with switch + assertNever, a shadow of exhaustiveness rather than the real thing.',
  },
];
