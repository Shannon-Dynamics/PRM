/**
 * The book's canonical structure: the single source of truth for chapter
 * numbers, slugs, and part membership. The landing page, the sidebar ordering
 * in content/chapters/meta.json, and every cross-chapter link agree with this.
 * Titles and difficulties come from the design docs (../Chapter-NN.md).
 */

export interface ChapterEntry {
  n: number;
  slug: string;
  title: string;
  blurb: string;
  difficulty: 'Foundational' | 'Intermediate' | 'Advanced';
}

export interface PartEntry {
  id: string;
  title: string;
  chapters: ChapterEntry[];
}

export const PARTS: PartEntry[] = [
  {
    id: 'PART I',
    title: 'Foundations — Robots, Worlds, and Configuration Space',
    chapters: [
      {
        n: 1,
        slug: 'ch01-robots-that-plan',
        title: 'Robots That Plan: The Piano Mover\'s Problem',
        blurb: 'Why motion planning is hard and worth it: the piano mover, the taxonomy of planners, and the three robots of this book.',
        difficulty: 'Foundational',
      },
      {
        n: 2,
        slug: 'ch02-the-lab',
        title: 'The Lab: Three Robots, Three Worlds, One Simulator',
        blurb: 'Rusty, Reach, and Hitch; the Apartment, the Workbench, and the Lot; collision queries and the deterministic simulator every chapter runs in.',
        difficulty: 'Intermediate',
      },
      {
        n: 3,
        slug: 'ch03-bug-algorithms',
        title: 'Bug Algorithms: Planning with a Contact Sensor',
        blurb: 'Bug1, Bug2, and Tangent Bug: provable sensor-based planning from almost no mathematics, and boundary following as curve tracing.',
        difficulty: 'Foundational',
      },
      {
        n: 4,
        slug: 'ch04-configuration-space-i',
        title: 'Configuration Space I: Obstacles in the Space of Configurations',
        blurb: 'Configurations, C-obstacles, the torus of a two-joint arm, Minkowski sums, and what a collision checker actually computes.',
        difficulty: 'Intermediate',
      },
      {
        n: 5,
        slug: 'ch05-configuration-space-ii',
        title: 'Configuration Space II: Topology, Manifolds, and Rigid Bodies',
        blurb: 'Homeomorphisms, manifolds, rigid-body configuration spaces, metrics, and the Manifold trait every planner is generic over.',
        difficulty: 'Intermediate',
      },
    ],
  },
  {
    id: 'PART II',
    title: 'Classical Planners — Search, Potentials, Roadmaps, Cells',
    chapters: [
      {
        n: 6,
        slug: 'ch06-graph-search',
        title: 'Graph Search for Planners',
        blurb: 'BFS, Dijkstra, A* with its proofs, D* for replanning, universal plans, and what complete and optimal really mean.',
        difficulty: 'Foundational',
      },
      {
        n: 7,
        slug: 'ch07-potential-functions',
        title: 'Potential Functions',
        blurb: 'Attractive and repulsive potentials, brushfire and wave-front, the local-minimum disease, and navigation functions that cure it.',
        difficulty: 'Intermediate',
      },
      {
        n: 8,
        slug: 'ch08-roadmaps-i',
        title: 'Roadmaps I: Visibility Graphs and the Generalized Voronoi Diagram',
        blurb: 'Visibility graphs and the generalized Voronoi diagram as a deformation retract, with the preimage theorem behind it.',
        difficulty: 'Intermediate',
      },
      {
        n: 9,
        slug: 'ch09-roadmaps-ii',
        title: 'Roadmaps II: Sensor-Based Roadmaps and Silhouettes',
        blurb: 'The generalized Voronoi graph built from range data, hierarchical and rod variants, and Canny\'s silhouette roadmap.',
        difficulty: 'Advanced',
      },
      {
        n: 10,
        slug: 'ch10-cell-decompositions',
        title: 'Cell Decompositions and Coverage',
        blurb: 'Trapezoidal and Morse decompositions, boustrophedon coverage, sensor-based coverage, and pursuit-evasion as a coda.',
        difficulty: 'Intermediate',
      },
    ],
  },
  {
    id: 'PART III',
    title: 'Sampling-Based Planning',
    chapters: [
      {
        n: 11,
        slug: 'ch11-prm',
        title: 'Probabilistic Roadmaps',
        blurb: 'Probabilistic roadmaps: samplers, kd-trees, local planners, narrow passages, and probabilistic completeness stated precisely.',
        difficulty: 'Intermediate',
      },
      {
        n: 12,
        slug: 'ch12-tree-planners',
        title: 'Tree Planners: EST, RRT, and Friends',
        blurb: 'EST, RRT and its Voronoi bias, RRT-Connect, SBL, SRT, and KPIECE: single-query planning as tree growth.',
        difficulty: 'Intermediate',
      },
      {
        n: 13,
        slug: 'ch13-optimal-sampling',
        title: 'Optimal Sampling-Based Planning',
        blurb: 'RRT*, PRM*, Informed RRT*, BIT*, FMT*: why RRT never improves and what asymptotic optimality costs.',
        difficulty: 'Advanced',
      },
      {
        n: 14,
        slug: 'ch14-many-bodies',
        title: 'Planning for Many Bodies: Multi-Robot, Manipulation, and Kinodynamic Previews',
        blurb: 'Composite configuration spaces, conflict-based search, manipulation modes, and a first forward-propagation planner for Hitch.',
        difficulty: 'Advanced',
      },
    ],
  },
  {
    id: 'PART IV',
    title: 'Planning with Uncertainty',
    chapters: [
      {
        n: 15,
        slug: 'ch15-kalman-observer',
        title: 'Kalman Filtering: The Geometric Observer',
        blurb: 'The Luenberger observer, the Kalman filter as its optimal gain, observability, and EKF localization through Choset\'s geometric lens.',
        difficulty: 'Intermediate',
      },
      {
        n: 16,
        slug: 'ch16-bayesian-localization',
        title: 'Bayesian Localization and Mapping',
        blurb: 'Recursive Bayesian localization, grid and particle posteriors, mapping with known poses, and replanning on a belief.',
        difficulty: 'Intermediate',
      },
    ],
  },
  {
    id: 'PART V',
    title: 'Dynamics, Trajectories, and Constraints',
    chapters: [
      {
        n: 17,
        slug: 'ch17-robot-dynamics',
        title: 'Robot Dynamics',
        blurb: 'Lagrangian dynamics of Reach, Christoffel symbols, Pfaffian constraints, and rigid-body rotation.',
        difficulty: 'Intermediate',
      },
      {
        n: 18,
        slug: 'ch18-trajectory-planning',
        title: 'Trajectory Planning',
        blurb: 'Path-velocity decomposition, the phase plane and its velocity-limit curve, zero-inertia points, and TOPP-RA.',
        difficulty: 'Advanced',
      },
      {
        n: 19,
        slug: 'ch19-trajectory-optimization',
        title: 'Trajectory Optimization and MPC',
        blurb: 'CHOMP, STOMP, TrajOpt, iLQR and MPC: trajectory optimization as the modern descendant of direct methods.',
        difficulty: 'Advanced',
      },
      {
        n: 20,
        slug: 'ch20-nonholonomic-i',
        title: 'Nonholonomic Systems I: Controllability',
        blurb: 'Vector fields, distributions, Lie brackets, Chow\'s theorem, and controllability tests for cars and mechanical systems.',
        difficulty: 'Advanced',
      },
      {
        n: 21,
        slug: 'ch21-nonholonomic-ii',
        title: 'Nonholonomic Systems II: Steering Cars, Trailers, and Underactuated Systems',
        blurb: 'Dubins and Reeds-Shepp, chained forms, differential flatness, trailers, kinodynamic trees, state lattices and hybrid A*.',
        difficulty: 'Advanced',
      },
    ],
  },
  {
    id: 'PART VI',
    title: 'Frontiers and Integration',
    chapters: [
      {
        n: 22,
        slug: 'ch22-learning-and-planning',
        title: 'Learning and Planning',
        blurb: 'Learned samplers, learned heuristics, neural and diffusion planners, benchmarked honestly against the classical baselines.',
        difficulty: 'Advanced',
      },
      {
        n: 23,
        slug: 'ch23-capstone',
        title: 'Capstone: A Planning Stack from Sensor to Motion',
        blurb: 'Three robots, one planning stack: sensor-based navigation, time-optimal arm trajectories, and trailer parking, running live.',
        difficulty: 'Advanced',
      },
    ],
  },
];

export const ALL_CHAPTERS: ChapterEntry[] = PARTS.flatMap((p) => p.chapters);

export const chapterBySlug = (slug: string) => ALL_CHAPTERS.find((c) => c.slug === slug);
export const chapterByNumber = (n: number) => ALL_CHAPTERS.find((c) => c.n === n);

/** Part label for a chapter number, e.g. "PART II". */
export function partOf(n: number): PartEntry | undefined {
  return PARTS.find((p) => p.chapters.some((c) => c.n === n));
}
