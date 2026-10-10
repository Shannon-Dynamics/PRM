/**
 * Numerical self-checks for Chapter 23 — `capstone`.
 *
 * The design doc's micro-example (F2 with the book's default rates: T_react,
 * the 0.750 m stopping distance, v_safe = 1.264 m/s, L_valid = 0.42 m) and the
 * exercises' derived numbers; D23.3's composition rule against brute force;
 * the guards' statistics against their closed forms (F3 from Chapter 4's
 * Jacobian, F4 from Chapter 20's trailer field); the supervisor's
 * exhaustiveness; and the mission itself: each stack completes its seeded
 * mission, every executed motion survives a dense recheck, each Failure Tour
 * injection is detected and recovered from, and `runMission` is bit-identical
 * across two runs with the same seed.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { REACH } from '../robots/reach';
import { HITCH_WITH_TRAILER } from '../robots/hitch';
import { svd2, jacobian2 } from '../cspace/jacobian';
import { DEFAULT_RATES, maxWalkerSpeed, minPlannerRate, replanningBudget, tReact } from './budget';
import { Guarantee, composedGuarantee, type StageInfo } from './stage';
import { armConditioning, hitchRate, jackknifeGuard, singularityGuard } from './guards';
import { MODE_KINDS, supervisorStep, type Mode, type SupervisorEvent } from './supervisor';
import { APPROACH_TICKS, MU_MIN, QDOT_MAX } from './reach';
import { APPROACH_LEN } from './worlds';
import { DEPENDENCIES } from './dependencies';
import { gridFromWorld } from '../search/grid';
import { BLUEPRINT } from './worlds';
import { RUSTY_CELL, RUSTY_INFLATION } from './rusty';

const RUSTY_GRID_CELLS = (() => {
  const g = gridFromWorld(BLUEPRINT, RUSTY_CELL, RUSTY_INFLATION);
  return g.nx * g.ny;
})();
import { formatMission, runMission, SEED_7_CHAOS, type MissionReport } from './mission';

const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;

export function runCh23Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // 1. The micro-example of §3, to the design doc's tolerance (1e-3).
  {
    const b = replanningBudget(DEFAULT_RATES, 0.6, 1.0, 2.0);
    const identity = b.vSafe * b.tReact + (b.vSafe * b.vSafe) / 2 - 2.0;
    const ok =
      close(b.tReact, 0.95, 1e-12) &&
      close(b.reactDistance, 0.57, 1e-12) &&
      close(b.brakeDistance, 0.18, 1e-12) &&
      close(b.stop, 0.75, 1e-12) &&
      close(b.margin, 1.25, 1e-12) &&
      close(b.vSafe, 1.264, 1e-3) &&
      close(b.lValid, 0.42, 1e-12) &&
      Math.abs(identity) < 1e-12 &&
      b.ok;
    push(
      'F2 micro-example: T_react 0.950 s, stop 0.750 m < 2.0 m, v_safe 1.264 m/s, L_valid 0.42 m',
      ok,
      `T ${b.tReact.toFixed(3)}  stop ${b.stop.toFixed(3)}  margin ${b.margin.toFixed(3)}  v_safe ${b.vSafe.toFixed(4)}  L ${b.lValid.toFixed(2)}`,
    );
  }

  // 2. The exercises' numbers come from the same function, and v_safe is the boundary of F2.
  {
    const walker = maxWalkerSpeed(DEFAULT_RATES);
    const fMin = minPlannerRate(DEFAULT_RATES, 1.0);
    // At exactly f_min, F2 holds with equality at v = 1 m/s.
    const atMin = replanningBudget({ ...DEFAULT_RATES, planHz: fMin }, 1.0, 1.0, 2.0);
    const above = replanningBudget(DEFAULT_RATES, 1.2642, 1.0, 2.0);
    const below = replanningBudget(DEFAULT_RATES, 1.2641, 1.0, 2.0);
    const ok = close(walker, 1.25 / 0.95, 1e-12) && close(fMin, 1 / 1.05, 1e-12) && close(atMin.stop, 2.0, 1e-12) && !above.ok && below.ok && close(tReact(DEFAULT_RATES), 0.95, 1e-12);
    push('Exercises 1 and 3: walker ≤ 1.316 m/s; replanner ≥ 0.952 Hz at 1 m/s; v_safe splits safe from unsafe', ok, `walker ${walker.toFixed(3)} m/s, f_min ${fMin.toFixed(3)} Hz`);
  }

  // 3. D23.3 against brute force over random stage lists.
  {
    const rng = new Rng(23);
    const roles: StageInfo['role'][] = ['source', 'rejecting', 'verified', 'unverified', 'executor', 'guard'];
    let bad = 0;
    for (let t = 0; t < 2000; t++) {
      const n = 1 + Math.floor(rng.next() * 6);
      const stages = Array.from({ length: n }, () => ({ role: roles[Math.floor(rng.next() * roles.length)], guarantee: Math.floor(rng.next() * 4) as Guarantee }));
      let expected: Guarantee;
      if (stages.some((s) => s.role === 'unverified')) expected = Guarantee.None;
      else {
        const rej = stages.filter((s) => s.role === 'rejecting').map((s) => s.guarantee as number);
        expected = rej.length ? (Math.min(...rej) as Guarantee) : Guarantee.None;
      }
      if (composedGuarantee(stages) !== expected) bad++;
    }
    push('D23.3: composed guarantee = min over rejecting stages, none with an unverified post-processor', bad === 0, `${bad} mismatches in 2000 random pipelines`);
  }

  // 4. F3 and F4: the guards fire exactly where their closed forms say.
  {
    let bad = 0;
    for (let k = 0; k <= 720; k++) {
      const th2 = -Math.PI + (k / 720) * 2 * Math.PI;
      const q = [0.3, th2];
      const mu = armConditioning(REACH, q).mu;
      if (!close(mu, Math.abs(Math.sin(th2)), 1e-12)) bad++;
      if (singularityGuard(REACH, q, MU_MIN).ok !== Math.abs(Math.sin(th2)) >= MU_MIN) bad++;
    }
    // σ_min at the threshold: the joint-rate amplification the guard admits (‖q̇‖ ≤ ‖ẋ‖/σ_min).
    const sMin = svd2(jacobian2(REACH, [0, Math.asin(MU_MIN)])).sigma[1];
    const d = HITCH_WITH_TRAILER.p.hitchLen!;
    const L = HITCH_WITH_TRAILER.p.wheelbase;
    let bad4 = 0;
    for (const phi of [-1, -0.4, -0.05, 0.05, 0.4, 1]) {
      for (const v of [-1, 1]) {
        const steer = 0.3 * Math.sign(phi);
        if (!close(hitchRate(phi, v, steer, L, d), (v / L) * Math.tan(steer) - (v / d) * Math.sin(phi), 1e-12)) bad4++;
        // Straight steering: reverse grows |φ|, forward shrinks it (the sign F4 is about).
        const grow = Math.sign(hitchRate(phi, v, 0, L, d)) === Math.sign(phi);
        if (grow !== v < 0) bad4++;
      }
      if (jackknifeGuard(phi, -1, 0.9).ok !== Math.abs(phi) <= 0.9 || !jackknifeGuard(phi, 1, 0.9).ok) bad4++;
    }
    // The design point: at μ_min the F3 bound on the approach's joint speed is under the joint-rate limit,
    // and the pulled-in target (r = 2 − δ = 1.85 m) sits at μ = 0.703.
    const xdot = APPROACH_LEN / (APPROACH_TICKS * 0.05);
    const bound = xdot / sMin;
    const r = 1.85;
    const muPulled = Math.sqrt(1 - ((r * r - 2) / 2) ** 2);
    const design = close(sMin, 0.1127, 1e-4) && close(xdot, 0.2, 1e-12) && bound < QDOT_MAX && close(bound, 1.775, 1e-3) && close(muPulled, 0.703, 1e-3);
    push('F3/F4: μ = l₁l₂|sin θ₂| guard and the Chapter 20 hitch-angle rate, reverse unstable', bad === 0 && bad4 === 0 && design, `σ_min at μ_min = ${sMin.toFixed(4)}, ‖ẋ‖/σ_min = ${bound.toFixed(3)} < ${QDOT_MAX} rad/s, μ after pull-in ${muPulled.toFixed(3)}; ${bad + bad4} mismatches`);
  }

  // 5. D23.4: every (mode, event) pair lands on a mode; terminal modes are absorbing.
  {
    const recover = { kind: 'TangentBug' } as const;
    const modes: Mode[] = [{ kind: 'Plan' }, { kind: 'Execute' }, { kind: 'Replan' }, { kind: 'Recover', recover }, { kind: 'Done' }, { kind: 'Failed', reason: 'NoPath', detail: '' }];
    const events: SupervisorEvent[] = [
      { kind: 'PlanReady' },
      { kind: 'NoPath', recover },
      { kind: 'NoPath', recover: null },
      { kind: 'Blocked', at: 0.3 },
      { kind: 'Arrived' },
      { kind: 'NextTask' },
      { kind: 'Singular', delta: 0.15 },
      { kind: 'Jackknife', untilPhi: 0.15 },
      { kind: 'Recovered' },
      { kind: 'Fault', reason: 'Collision', detail: '' },
    ];
    let bad = 0;
    for (const m of modes) {
      for (const e of events) {
        const n = supervisorStep(m, e);
        if (!MODE_KINDS.includes(n.kind)) bad++;
        if (m.kind === 'Failed' && n !== m) bad++;
        if (m.kind === 'Done' && e.kind !== 'NextTask' && n !== m) bad++;
      }
    }
    push('D23.4: the supervisor is total over 6 modes × 10 events, Failed absorbing, Done leaves only on NextTask', bad === 0, `${bad} violations`);
  }

  // 6–9: the missions.
  let plain: MissionReport | null = null;
  let chaos: MissionReport | null = null;
  try {
    plain = runMission({ seed: 7 });
    chaos = runMission({ seed: 7, chaos: SEED_7_CHAOS });
  } catch (e) {
    push('missions run', false, String(e));
    return out;
  }
  {
    const r = plain;
    const ok =
      r.rusty!.done && r.reach!.done && r.reach!.picks === 3 && r.hitch!.done && r.hitch!.posErr < 0.04 && r.hitch!.headErr < (1.8 * Math.PI) / 180 &&
      r.rusty!.furniture.join(' + ') === 'a cart in the corridor + a basket in the bedroom';
    push(
      'seed-7 mission: Rusty reaches the bedroom, Reach makes 3 picks, Hitch parks in bay 3',
      ok,
      `Rusty ${r.rusty!.pathLength.toFixed(2)} m vs ${r.rusty!.optimalLength.toFixed(2)} m (ratio ${r.rusty!.ratio.toFixed(2)}), ${r.rusty!.replans} replans; Reach T ${r.reach!.durationBefore.toFixed(2)} → ${r.reach!.durationAfter.toFixed(2)} s; Hitch ${r.hitch!.posErr.toExponential(1)} m, ${r.hitch!.cusps} cusp(s), max |φ| ${r.hitch!.maxPhi.toFixed(2)}`,
    );
  }
  {
    const okP = plain.collisionFree.rusty && plain.collisionFree.reach && plain.collisionFree.hitch;
    const okC = chaos.collisionFree.rusty && chaos.collisionFree.reach && chaos.collisionFree.hitch;
    const contacts = plain.rusty!.collisions + plain.reach!.collisions + plain.hitch!.collisions + chaos.rusty!.collisions + chaos.reach!.collisions + chaos.hitch!.collisions;
    push('every executed motion is collision-free by dense recheck (plain and chaos missions)', okP && okC && contacts === 0, `plain ${JSON.stringify(plain.collisionFree)}, chaos ${JSON.stringify(chaos.collisionFree)}`);
  }
  {
    const ev = chaos.events;
    const has = (robot: string, re: RegExp) => ev.some((e) => e.robot === robot && re.test(e.text));
    const door = has('rusty', /swings shut/) && has('rusty', /D\* Lite repaired 380 vertices/) && chaos.rusty!.done && RUSTY_GRID_CELLS === 10800;
    const arm = chaos.reach!.guardFired && has('reach', /singularity guard/) && has('reach', /PullGoalInward/) && chaos.reach!.done;
    const trailer = chaos.hitch!.guardFired && has('hitch', /jackknife guard/) && has('hitch', /recovered/) && chaos.hitch!.done;
    push(
      'Failure Tour: blocked corridor → D* Lite repair; singular goal → μ guard → pull inward; jackknife → guard → pull forward → parked',
      door && arm && trailer,
      `door ${door}, arm ${arm}, trailer ${trailer}; Rusty ratio ${chaos.rusty!.ratio.toFixed(2)}, Hitch max |φ| ${chaos.hitch!.maxPhi.toFixed(2)} < φ_max`,
    );
  }
  {
    const again = runMission({ seed: 7, chaos: SEED_7_CHAOS });
    const a = JSON.stringify(chaos);
    const b = JSON.stringify(again);
    push('runMission is bit-identical across two runs with the same seed', a === b, `${a.length} bytes of report, ${a === b ? 'identical' : 'DIFFERENT'}`);
  }

  // The worked example's printout, as the chapter prints it — every line, so every number in it is pinned.
  {
    const lines = formatMission(chaos);
    const expected = [
      't_react 0.950 s  stop 0.750 m  margin 1.250 m  v_safe 1.264 m/s  l_valid 0.42 m',
      '  2.20 s  reach  pick 1 done at (-1.50, -0.55)',
      '  2.50 s  hitch  chaos: Reverse Hard — full lock, full reverse',
      '  4.00 s  reach  chaos: the next pick is pushed out to full stretch',
      '  4.70 s  hitch  jackknife guard: |φ| = 0.91 rad > φ_warn = 0.9 while reversing',
      '  5.00 s  reach  pick 2 done at (1.55, -0.55)',
      '  5.55 s  reach  singularity guard: μ(q_goal) = 0.000 < μ_min = 0.25 (σ_min = 0.000)',
      '  7.20 s  hitch  recovered: |φ| = 0.15 < φ_ok = 0.15; replanning from here',
      '  7.25 s  hitch  trailer filter rejects hybrid A* (cell 0.5 m, reverse ×1): |φ| reaches 0.85 > φ_plan',
      '  7.30 s  hitch  trailer filter rejects hybrid A* (cell 0.4 m, reverse ×3): |φ| reaches 0.85 > φ_plan',
      '  7.35 s  hitch  hybrid A* (cell 0.5 m, reverse ×1, via (12.4, 4.2)): 11.02 m, 2 cusps, 2 expansions, max |φ| 0.69 rad',
      '  7.90 s  reach  pick 3 done at (-0.50, 1.78)',
      '  9.00 s  rusty  chaos: the study–bedroom door swings shut',
      ' 11.00 s  rusty  D* Lite repaired 3 times, 24–380 vertices each (until 12.00 s)',
      ' 18.35 s  hitch  parked: 0.000 m, 0.00°',
      ' 18.50 s  rusty  D* Lite repaired 7 times, 14–117 vertices each (until 24.50 s)',
      ' 24.90 s  rusty  D23.2: occupied cell 0.40 m ahead inside L_valid = 0.42 m — braking',
      ' 25.00 s  rusty  D* Lite repaired 3 times, 14–32 vertices each (until 26.50 s)',
      'rusty  Done: driven 14.54 m vs 11.02 m final-map optimum (ratio 1.32), 13 repairs',
      'reach  Done: 3 picks; pick 1 cost 2.14 rad, t_f 0.97 s → 0.93 s after CHOMP; max |τ|/τ_max 1.00 at the grid nodes',
      'hitch  Done: parked within 0.0 cm / 0.0°, 3 cusps, max |φ| 0.91 rad',
      'dense recheck: rusty true, reach true, hitch true',
    ];
    const plainTail = formatMission(plain).slice(-4);
    const plainExpected = [
      'rusty  Done: driven 10.76 m vs 10.99 m final-map optimum (ratio 0.98), 3 repairs',
      'reach  Done: 3 picks; pick 1 cost 2.14 rad, t_f 0.97 s → 0.93 s after CHOMP; max |τ|/τ_max 1.00 at the grid nodes',
      'hitch  Done: parked within 0.0 cm / 0.0°, 1 cusps, max |φ| 0.77 rad',
      'dense recheck: rusty true, reach true, hitch true',
    ];
    const bad = expected.filter((e, i) => lines[i] !== e).length + plainExpected.filter((e, i) => plainTail[i] !== e).length;
    push('worked example (`--seed 7`, with and without chaos): every printed line pinned', bad === 0 && lines.length === expected.length, bad ? `first mismatch: ${lines.find((l, i) => l !== expected[i]) ?? plainTail.find((l, i) => l !== plainExpected[i])}` : `${lines.length} + ${plainTail.length} lines`);
  }

  // 10. Degradations are causal (w23.2): the throttled replanner leans on D23.2; each switched-off stage
  //     fails the way its contract card says.
  {
    const thr = runMission({ seed: 7, robots: ['rusty'], chaos: SEED_7_CHAOS, rates: { ...DEFAULT_RATES, planHz: 0.5 } });
    const stopped = thr.events.some((e) => /D23\.2/.test(e.text));
    const budgetOk = close(thr.budget.lValid, 1.32, 1e-12) && close(thr.budget.tReact, 2.45, 1e-12) && close(thr.budget.stop, 1.65, 1e-12) && thr.budget.ok;
    const off = (id: string, robot: 'rusty' | 'reach' | 'hitch', from = 3000) => runMission({ seed: 7, robots: [robot], chaos: SEED_7_CHAOS, disabled: [{ id, from }], tMaxMs: 40000 });
    const plan = off('rusty.plan', 'rusty', 10000);
    const scale = off('reach.scale', 'reach');
    const guard = off('reach.guard', 'reach', 0);
    const jack = off('hitch.guard', 'hitch', 0);
    const ok =
      stopped && thr.rusty!.done && thr.collisionFree.rusty && budgetOk &&
      plan.rusty!.mode === 'Replan' && plan.events.some((e) => /D23\.2/.test(e.text)) && plan.collisionFree.rusty &&
      scale.reach!.mode === 'Failed(PathNotTrajectory)' &&
      guard.reach!.mode === 'Failed(JointRate)' &&
      jack.hitch!.mode === 'Failed(Jackknife)';
    push(
      'degradations are causal: 0.5 Hz replanner braked by D23.2; D* Lite off → stuck in Replan; time scaling off → Path refused; guards off → JointRate / Jackknife',
      ok,
      `throttled T_react ${thr.budget.tReact.toFixed(2)} s, stop ${thr.budget.stop.toFixed(2)} m, L_valid ${thr.budget.lValid.toFixed(2)} m; ${plan.rusty!.mode}, ${scale.reach!.mode}, ${guard.reach!.mode}, ${jack.hitch!.mode}`,
    );
  }

  // 11. The dependency table covers every chapter once, and every artifact row names an import.
  {
    const chapters = DEPENDENCIES.filter((d) => d.chapter.startsWith('Ch.')).map((d) => Number(d.chapter.slice(4)));
    const all = Array.from({ length: 22 }, (_, i) => i + 1);
    const ok = all.every((c) => chapters.filter((x) => x === c).length === 1) && DEPENDENCIES.every((d) => (d.kind === 'artifact') === d.imports.length > 0);
    push('dependency table: Chapters 1–22 once each; artifact ⇔ an imported lib module', ok, `${DEPENDENCIES.filter((d) => d.kind === 'artifact').length} artifact rows, ${DEPENDENCIES.filter((d) => d.kind === 'background').length} background`);
  }

  return out;
}
