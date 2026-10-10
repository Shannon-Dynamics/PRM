/**
 * `trajopt` — Chapter 19: trajectory optimization and MPC. The shared
 * Gauss–Newton/Levenberg–Marquardt optimizer (imported by Chapter 21), CHOMP's
 * covariant gradient on a distance field, STOMP's seeded rollouts, a
 * sequential-convex recipe with hinge collision penalties, iLQR/DDP over
 * Chapter 17's dynamics, and a receding-horizon loop.
 */
export * from './optim';
export * from './field';
export * from './chomp';
export * from './stomp';
export * from './sco';
export * from './models';
export * from './ilqr';
export * from './mpc';
export * from './micro';
