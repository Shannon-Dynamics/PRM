/**
 * `capstone` — Chapter 23: a planning stack from sensor to motion.
 *
 * Nothing here is a new planner. The stage/contract types (D23.1), the
 * composed-guarantee rule (D23.3), the replanning budget (F2), the guards
 * (F3, F4, D23.2), the supervisor's exhaustive mode machine (D23.4), the
 * deterministic round-robin scheduler, and three pipelines that import
 * Chapters 2–21 — Rusty in the Apartment, Reach on the Workbench, Hitch in
 * the Lot — plus the seeded `runMission` the checks pin.
 */
export * from './bus';
export * from './stage';
export * from './budget';
export * from './guards';
export * from './supervisor';
export * from './worlds';
export * from './rusty';
export * from './reach';
export * from './hitch';
export * from './mission';
export * from './dependencies';
