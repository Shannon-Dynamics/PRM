/**
 * The replanning budget (F2) and the validity horizon (D23.2).
 *
 * A newly blocked corridor, first trusted at distance d_detect, is handled iff
 *
 *     v_max · T_react + v_max² / (2 a_max) < d_detect,
 *     T_react = ς_scan + ς_bel + ς_map + τ_plan + τ_ctrl.
 *
 * Worst case, every message in the chain was produced just after its consumer
 * last read, so each stage adds one full period of staleness; the robot keeps
 * driving at v_max for that long, then brakes. The validity horizon is the
 * prefix the robot may drive before the next plan can possibly arrive:
 * L_valid = v_max (τ_plan + ς_map), re-checked against the newest map every
 * control tick.
 */

/** Stage rates in hertz. Periods (and worst-case staleness) are their reciprocals. */
export interface RateTable {
  scanHz: number;
  beliefHz: number;
  mapHz: number;
  planHz: number;
  ctrlHz: number;
}

/** The book's default rates (the design doc's micro-example). */
export const DEFAULT_RATES: RateTable = { scanHz: 10, beliefHz: 10, mapHz: 5, planHz: 2, ctrlHz: 20 };

/** Rusty's speed and braking limits and the trusted sensing distance of the micro-example. */
export const RUSTY_V_MAX = 0.6;
export const RUSTY_A_MAX = 1.0;
export const D_DETECT = 2.0;

export interface BudgetReport {
  /** ς_scan, ς_bel, ς_map, τ_plan, τ_ctrl in seconds. */
  terms: { scan: number; bel: number; map: number; plan: number; ctrl: number };
  tReact: number;
  /** v_max T_react. */
  reactDistance: number;
  /** v_max² / 2a_max. */
  brakeDistance: number;
  /** reactDistance + brakeDistance. */
  stop: number;
  margin: number;
  ok: boolean;
  /** Largest v with v T_react + v²/2a = d_detect. */
  vSafe: number;
  /** v_max (τ_plan + ς_map). */
  lValid: number;
}

export function tReact(r: RateTable): number {
  return 1 / r.scanHz + 1 / r.beliefHz + 1 / r.mapHz + 1 / r.planHz + 1 / r.ctrlHz;
}

/**
 * `replanning_budget` of the design doc. The safe speed is the positive root of
 * v²/(2a) + T v − d = 0, i.e. v = a (−T + √(T² + 2d/a)).
 */
export function replanningBudget(r: RateTable, vMax = RUSTY_V_MAX, aMax = RUSTY_A_MAX, dDetect = D_DETECT): BudgetReport {
  const terms = { scan: 1 / r.scanHz, bel: 1 / r.beliefHz, map: 1 / r.mapHz, plan: 1 / r.planHz, ctrl: 1 / r.ctrlHz };
  const T = terms.scan + terms.bel + terms.map + terms.plan + terms.ctrl;
  const reactDistance = vMax * T;
  const brakeDistance = (vMax * vMax) / (2 * aMax);
  const stop = reactDistance + brakeDistance;
  const vSafe = aMax * (-T + Math.sqrt(T * T + (2 * dDetect) / aMax));
  return {
    terms,
    tReact: T,
    reactDistance,
    brakeDistance,
    stop,
    margin: dDetect - stop,
    ok: stop < dDetect,
    vSafe,
    lValid: vMax * (terms.plan + terms.map),
  };
}

/**
 * Exercise 1's variant: an obstacle approaching at v_obs closes the gap during
 * the reaction time too, so the left side gains v_obs T_react. Returns the
 * fastest walker the rates tolerate.
 */
export function maxWalkerSpeed(r: RateTable, vMax = RUSTY_V_MAX, aMax = RUSTY_A_MAX, dDetect = D_DETECT): number {
  const b = replanningBudget(r, vMax, aMax, dDetect);
  return Math.max(0, b.margin / b.tReact);
}

/**
 * Exercise 3: the slowest replanner (in hertz) that still satisfies F2 at
 * speed `vMax`, everything else at its table rate. Solve
 * v (T₀ + 1/f) + v²/2a = d for f, where T₀ is T_react without τ_plan.
 */
export function minPlannerRate(r: RateTable, vMax: number, aMax = RUSTY_A_MAX, dDetect = D_DETECT): number {
  const t0 = tReact(r) - 1 / r.planHz;
  const tauMax = (dDetect - (vMax * vMax) / (2 * aMax)) / vMax - t0;
  return tauMax > 0 ? 1 / tauMax : Infinity;
}

/** The printed budget line of the worked example (`--seed 7`). */
export function budgetLine(b: BudgetReport): string {
  return `t_react ${b.tReact.toFixed(3)} s  stop ${b.stop.toFixed(3)} m  margin ${b.margin.toFixed(3)} m  v_safe ${b.vSafe.toFixed(3)} m/s  l_valid ${b.lValid.toFixed(2)} m`;
}
