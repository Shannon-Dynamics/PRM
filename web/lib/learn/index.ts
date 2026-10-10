/**
 * `learn` — Chapter 22, Learning and Planning.
 *
 * Learning replaces the proposal, never the verifier. A tiny MLP with manual
 * backprop; a conditional learned sampler that implements Chapter 11's
 * `Sampler` and is only ever shipped inside Chapter 11's `Hybrid` with a
 * uniform floor; a learned cost-to-go for Chapter 6's A* with its ε measured,
 * not assumed; an MPNet-style proposal + verifier planner whose guarantee is
 * Chapter 12's RRT-Connect; a one-dimensional toy diffusion planner; and the
 * benchmark harness behind the Honest Benchmark Board.
 */
export * from './mlp';
export * from './info';
export * from './worlds';
export * from './torus-grid';
export * from './sampler';
export * from './heuristic';
export * from './verify';
export * from './neural-planner';
export * from './diffusion';
export * from './data';
export * from './bench';
export * from './models';
