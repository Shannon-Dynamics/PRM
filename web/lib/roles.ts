/**
 * The book's color roles, as plain data (no React, no 'use client'), so both
 * server-rendered pages and client widgets can import the same lists.
 *
 * Two families share one mechanism: the planning roles this book is about
 * (`--rm-*`) and the estimation roles imported from the sister book for the
 * chapters where a belief appears (`--pr-*`).
 */
export type PlanningRole = 'start' | 'goal' | 'obstacle' | 'cobstacle' | 'graph' | 'path' | 'robot';
export type EstimationRole = 'prior' | 'prediction' | 'measurement' | 'posterior' | 'truth';
export type Role = PlanningRole | EstimationRole;

export const PLANNING_ROLES: readonly PlanningRole[] = ['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot'];
export const ESTIMATION_ROLES: readonly EstimationRole[] = ['prior', 'prediction', 'measurement', 'posterior', 'truth'];
export const ALL_ROLES: readonly Role[] = [...PLANNING_ROLES, ...ESTIMATION_ROLES];

/** The CSS custom property carrying a role's color, e.g. `var(--rm-path)`. */
export function roleVar(role: Role): string {
  return (ESTIMATION_ROLES as readonly string[]).includes(role) ? `var(--pr-${role})` : `var(--rm-${role})`;
}
