'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawParticles, drawPath, drawRobot, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Pose2 } from '@/lib/geom/se2';
import { APARTMENT } from '@/lib/sim/world';
import {
  GridBelief,
  LOCALIZATION_PROXIMITY,
  ParticleBelief,
  recordRun,
  type RunLog,
} from '@/lib/estimate';

/**
 * w16.1 — Grid vs Particles.
 *
 * One recorded Apartment run — seeded odometry and 36-beam scans from the
 * Chapter 2 simulator — is replayed into two localizers that start knowing
 * nothing: Choset's Algorithm 16 on a grid over SE(2) (shift-and-smear
 * prediction, the one-byte expected-distance table of §9.1.5) and Algorithms
 * 17–18 as a particle cloud. Same recursion, same sensor model, same data; the
 * only thing that differs is how the posterior is written down.
 *
 * The headline slider is a *budget*: it moves grid resolution and particle
 * count together along settings that cost about the same per step, so the
 * reader compares representations, not price tags. The cost meters underneath
 * show what that budget actually buys.
 */

interface Budget {
  cellSize: number;
  nTheta: number;
  particles: number;
}

/** Equal-cost rungs: finer grid ↔ more particles. */
const BUDGETS: readonly Budget[] = [
  { cellSize: 0.6, nTheta: 12, particles: 150 },
  { cellSize: 0.5, nTheta: 16, particles: 300 },
  { cellSize: 0.4, nTheta: 18, particles: 600 },
  { cellSize: 0.35, nTheta: 24, particles: 1200 },
  { cellSize: 0.3, nTheta: 30, particles: 2000 },
];

const STEPS = 90;
const KIDNAP_TO: Pose2 = { x: 10.0, y: 2.0, theta: Math.PI };
/** Burgard et al.'s restart threshold on the per-beam average likelihood p̃. */
const P_TILDE_RESTART = 0.08;

interface Params {
  budget: number;
  decouple: boolean;
  gridBudget: number;
  particleBudget: number;
  sigma: number;
  every: number;
  autoRestart: boolean;
}

interface State {
  seed: number;
  rng: Rng;
  log: RunLog;
  k: number;
  grid: GridBelief;
  pf: ParticleBelief;
  gridMs: number[];
  pfMs: number[];
  gridP: number[];
  pfP: number[];
  gridErr: number[];
  pfErr: number[];
  truthTrail: Pose2[];
  kidnappedAt: number | null;
  restarts: number;
}

const DEFAULT_PARAMS: Params = { budget: 2, decouple: false, gridBudget: 2, particleBudget: 2, sigma: LOCALIZATION_PROXIMITY.sigma, every: 3, autoRestart: true };

/**
 * A grid belief's expected-distance table is thousands of ray casts; build it
 * once per (resolution, sensor) and reset the belief itself on every replay.
 */
const gridCache = new Map<string, GridBelief>();
function gridFor(b: Budget, sigma: number): GridBelief {
  const key = `${b.cellSize}|${b.nTheta}|${sigma}`;
  let g = gridCache.get(key);
  if (!g) {
    g = new GridBelief(APARTMENT, { spec: { cellSize: b.cellSize, nTheta: b.nTheta, clearance: 0.12 }, beamStride: 1, sensor: { ...LOCALIZATION_PROXIMITY, sigma } });
    gridCache.set(key, g);
  }
  g.restart();
  g.pTilde = 1;
  g.lastStepMs = 0;
  return g;
}

export function GridVsParticles() {
  const [params, setParams] = useState<Params>(DEFAULT_PARAMS);
  const gridBudget = BUDGETS[params.decouple ? params.gridBudget : params.budget];
  const particleBudget = BUDGETS[params.decouple ? params.particleBudget : params.budget];

  const init = useCallback(
    (seed: number): State => {
      const rng = new Rng(seed + 7);
      const log = recordRun(APARTMENT, { seed, steps: STEPS });
      return {
        seed,
        rng,
        log,
        k: 0,
        grid: gridFor(gridBudget, params.sigma),
        pf: new ParticleBelief(APARTMENT, rng, { n: particleBudget.particles, every: params.every, sensor: { ...LOCALIZATION_PROXIMITY, sigma: params.sigma } }),
        gridMs: [],
        pfMs: [],
        gridP: [],
        pfP: [],
        gridErr: [],
        pfErr: [],
        truthTrail: [log.start],
        kidnappedAt: null,
        restarts: 0,
      };
    },
    [gridBudget, particleBudget, params.sigma, params.every],
  );

  const step = useCallback(
    (s: State): State => {
      if (s.k >= s.log.steps.length) return s;
      const rec = s.log.steps[s.k];
      const { grid, pf, rng } = s;

      // Alg. 16: shift-and-smear, then multiply by the table-driven likelihood.
      const g0 = grid.lastStepMs;
      grid.predict(rec.u);
      const gridP = grid.correct(grid.scanBinsFor(rec.scan, s.log.angles));
      let restarts = s.restarts;
      if (params.autoRestart && gridP < P_TILDE_RESTART && s.k > 5) {
        // Burgard et al. [C ref. 82]: when p̃ dives, start global localization again.
        grid.restart();
        restarts++;
      }
      const gridMs = grid.lastStepMs - g0;

      // Algs. 17–18: sample the motion, weight by the scan, resample.
      const p0 = pf.lastStepMs;
      pf.predict(rec.u, rng);
      const pfP = pf.correct(rec.scan, s.log.angles);
      pf.resample(rng);
      const pfMs = pf.lastStepMs - p0;

      const gm = grid.modePose();
      const pm = pf.modePose();
      const keep = <T,>(a: T[], v: T) => [...a, v].slice(-STEPS);
      return {
        ...s,
        k: s.k + 1,
        restarts,
        gridMs: keep(s.gridMs, gridMs),
        pfMs: keep(s.pfMs, pfMs),
        gridP: keep(s.gridP, gridP),
        pfP: keep(s.pfP, pfP),
        gridErr: keep(s.gridErr, Math.hypot(gm.x - rec.truth.x, gm.y - rec.truth.y)),
        pfErr: keep(s.pfErr, Math.hypot(pm.x - rec.truth.x, pm.y - rec.truth.y)),
        truthTrail: keep(s.truthTrail, rec.truth),
        kidnappedAt: rec.kidnapped ? s.k : s.kidnappedAt,
      };
    },
    [params.autoRestart],
  );

  const sim = useSimulation<State>({ init, step, fps: 4, initialSeed: 1601, maxTicks: STEPS, loop: true });

  // A new budget or sensor model means new localizers: replay from the start.
  const firstInit = useRef(true);
  const { reset } = sim;
  useEffect(() => {
    if (firstInit.current) {
      firstInit.current = false;
      return;
    }
    reset();
  }, [reset]);

  /**
   * Kidnap: re-record the same seeded run with the truth carried to room C at the
   * current step. Everything before this step is bit-identical (the seed is the
   * same), so neither localizer notices anything until the next scan arrives.
   */
  const kidnap = useCallback(() => {
    sim.setState((s) => {
      if (s.k >= STEPS - 5) return s;
      const log = recordRun(APARTMENT, { seed: s.seed, steps: STEPS, kidnapAt: s.k, kidnapTo: KIDNAP_TO });
      return { ...s, log };
    });
  }, [sim]);

  const truth = sim.state.k > 0 ? sim.state.log.steps[sim.state.k - 1].truth : sim.state.log.start;

  const drawCommon = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, s: State) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      drawWorld(ctx, v, APARTMENT, p);
      drawPath(ctx, v, s.truthTrail, p.truth, { dashed: true, lineWidth: 1.2 });
    },
    [],
  );

  const drawMarkers = useCallback((ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, mean: { x: number; y: number }, mode: Pose2, truthPose: Pose2) => {
    // Mean (9.17): a ring with a cross — it may sit in a wall, which is the point.
    ctx.save();
    ctx.strokeStyle = p.posterior;
    ctx.lineWidth = 1.6;
    const r = Math.max(5, sl(v, 0.18));
    const mx = sx(v, mean.x);
    const my = sy(v, mean.y);
    ctx.beginPath();
    ctx.arc(mx, my, r, 0, Math.PI * 2);
    ctx.moveTo(mx - r, my);
    ctx.lineTo(mx + r, my);
    ctx.moveTo(mx, my - r);
    ctx.lineTo(mx, my + r);
    ctx.stroke();
    ctx.restore();
    // Mode: a filled robot glyph; truth: the hollow gray one.
    drawRobot(ctx, v, mode, p.posterior, 0.26);
    drawRobot(ctx, v, truthPose, p.truth, 0.26, { filled: false });
  }, []);

  const drawGridPanel = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      const s = sim.state;
      drawCommon(ctx, v, p, s);
      const g = s.grid.grid;
      const m = g.marginalXY();
      let pmax = 0;
      for (let i = 0; i < m.length; i++) if (m[i] > pmax) pmax = m[i];
      if (pmax > 0) {
        ctx.save();
        ctx.fillStyle = p.posterior;
        const w = sl(v, g.cellSize);
        for (let j = 0; j < g.ny; j++) {
          for (let i = 0; i < g.nx; i++) {
            const q = m[j * g.nx + i];
            if (q <= pmax * 0.02) continue;
            const [cx, cy] = g.cellCenterXY(i, j);
            ctx.globalAlpha = 0.08 + 0.82 * Math.sqrt(q / pmax);
            ctx.fillRect(sx(v, cx) - w / 2, sy(v, cy) - w / 2, w, w);
          }
        }
        ctx.restore();
      }
      drawMarkers(ctx, v, p, s.grid.meanXY(), s.grid.modePose(), truth);
      label(ctx, `grid ${g.nx}×${g.ny}×${g.nTheta}  ·  Alg. 16`, sx(v, 0.2), sy(v, 9) - 12, p.ink, { size: 11 });
    },
    [sim.state, truth, drawCommon, drawMarkers],
  );

  const drawParticlePanel = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      const s = sim.state;
      drawCommon(ctx, v, p, s);
      drawParticles(ctx, v, s.pf.pf.particles, p.posterior, { showHeading: s.pf.size <= 600, maxRadius: 2.2 });
      drawMarkers(ctx, v, p, s.pf.meanXY(), s.pf.modePose(), truth);
      label(ctx, `${s.pf.size} particles  ·  Algs. 17–18`, sx(v, 0.2), sy(v, 9) - 12, p.ink, { size: 11 });
    },
    [sim.state, truth, drawCommon, drawMarkers],
  );

  const stats = useMemo(() => {
    const s = sim.state;
    const last = (a: number[]) => a[a.length - 1] ?? 0;
    const avg = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
    return {
      grid: { ms: avg(s.gridMs.slice(-10)), kb: s.grid.bytes / 1024, err: last(s.gridErr), H: s.grid.entropyBits(), p: last(s.gridP) },
      pf: { ms: avg(s.pfMs.slice(-10)), kb: s.pf.bytes / 1024, err: last(s.pfErr), H: s.pf.entropyBits(), p: last(s.pfP) },
    };
  }, [sim.state]);

  const aria = 'The Apartment floorplan drawn twice. Left: a purple heat map of a grid posterior over position. Right: a purple particle cloud. On both, a gray hollow robot marks the truth, a filled purple robot the mode, and a crossed ring the mean.';

  return (
    <WidgetFrame
      id="w16.1"
      title="Grid vs Particles"
      teaches="The particle filter is not a cheaper grid. At equal cost the two fail differently: the grid blurs, the cloud starves, and only one of them recovers from a kidnap without being told."
      colorKey={['posterior', 'truth']}
      wide
      caption={
        <>
          One recorded run (seed shown) replayed into Algorithm 16 (left) and Algorithms 17–18 (right); both start uniform over
          free space. The filled purple robot is each posterior&rsquo;s <em>mode</em>, the crossed ring its <em>mean</em> (eq. 9.17), the
          hollow gray robot the truth. During the bimodal phase — rooms A and C look alike from inside — watch the mean sit in a
          wall. Move the budget slider: the grid gets coarser and smears; the cloud gets thinner and, below a few hundred
          particles, can starve on a kidnap. Press <em>kidnap</em>: the grid&rsquo;s p̃ dives and (with auto-restart) it begins again
          from uniform; the plain cloud has nothing where the robot now is.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-0 border-b border-fd-border md:grid-cols-2 md:divide-x md:divide-fd-border">
        <SimCanvas world={APARTMENT.bounds} draw={drawGridPanel} deps={[sim.tick, sim.state]} aspect={12.8 / 9.8} padding={0.4} ariaLabel={aria} />
        <SimCanvas world={APARTMENT.bounds} draw={drawParticlePanel} deps={[sim.tick, sim.state]} aspect={12.8 / 9.8} padding={0.4} ariaLabel={aria} />
      </div>
      <div className="grid grid-cols-2 divide-x divide-fd-border border-b border-fd-border">
        <CostMeter title="grid" ms={stats.grid.ms} kb={stats.grid.kb} err={stats.grid.err} H={stats.grid.H} pTilde={sim.state.gridP} restarts={sim.state.restarts} />
        <CostMeter title="particles" ms={stats.pf.ms} kb={stats.pf.kb} err={stats.pf.err} H={stats.pf.H} pTilde={sim.state.pfP} />
      </div>
      <ControlPanel columns={2}>
        {params.decouple ? (
          <>
            <Slider
              label="Grid resolution"
              role="posterior"
              value={params.gridBudget}
              min={0}
              max={BUDGETS.length - 1}
              step={1}
              format={(i) => `${BUDGETS[i].cellSize} m × ${(360 / BUDGETS[i].nTheta).toFixed(0)}°`}
              onChange={(v) => setParams((q) => ({ ...q, gridBudget: v }))}
            />
            <Slider
              label="Particles"
              role="posterior"
              value={params.particleBudget}
              min={0}
              max={BUDGETS.length - 1}
              step={1}
              format={(i) => `${BUDGETS[i].particles}`}
              onChange={(v) => setParams((q) => ({ ...q, particleBudget: v }))}
            />
          </>
        ) : (
          <Slider
            label="Budget (grid resolution ↔ particle count, equal cost)"
            role="posterior"
            value={params.budget}
            min={0}
            max={BUDGETS.length - 1}
            step={1}
            format={(i) => `${BUDGETS[i].cellSize} m × ${(360 / BUDGETS[i].nTheta).toFixed(0)}°  |  ${BUDGETS[i].particles} particles`}
            onChange={(v) => setParams((q) => ({ ...q, budget: v, gridBudget: v, particleBudget: v }))}
          />
        )}
        <ButtonRow>
          <ActionButton onClick={kidnap} emphasis>
            Kidnap Rusty to room C
          </ActionButton>
          <span className="font-mono text-[0.7rem] text-fd-muted-foreground tabular-nums">
            {sim.state.kidnappedAt !== null ? `kidnapped at t=${sim.state.kidnappedAt}` : 'not kidnapped'}
          </span>
        </ButtonRow>
      </ControlPanel>
      <details className="border-t border-fd-border px-3 py-2">
        <summary className="cursor-pointer font-ui text-[0.72rem] font-medium text-fd-muted-foreground">More knobs: decouple, sensor width, beam subsampling, restart monitor</summary>
        <div className="grid grid-cols-1 gap-x-5 gap-y-2.5 pt-2 sm:grid-cols-2">
          <Toggle label="Decouple grid resolution and particle count" checked={params.decouple} onChange={(d) => setParams((q) => ({ ...q, decouple: d }))} />
          <Toggle label="Grid auto-restart when p̃ dives (Burgard et al.)" role="posterior" checked={params.autoRestart} onChange={(a) => setParams((q) => ({ ...q, autoRestart: a }))} />
          <Slider
            label="Sensor-model σ (hit peak width; 0.08 m is the raw laser)"
            role="measurement"
            value={params.sigma}
            min={0.08}
            max={0.5}
            step={0.01}
            unit="m"
            onChange={(v) => setParams((q) => ({ ...q, sigma: v }))}
          />
          <Slider
            label="Beam subsampling (use every n-th of 36 beams)"
            value={params.every}
            min={1}
            max={6}
            step={1}
            format={(n) => `every ${n}  (${Math.ceil(36 / n)} beams)`}
            onChange={(v) => setParams((q) => ({ ...q, every: v }))}
          />
        </div>
      </details>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => sim.reseed()}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}

function CostMeter({ title, ms, kb, err, H, pTilde, restarts }: { title: string; ms: number; kb: number; err: number; H: number; pTilde: number[]; restarts?: number }) {
  const W = 120;
  const Hh = 22;
  const n = pTilde.length;
  const pts = pTilde.map((p, i) => `${(i / Math.max(1, n - 1)) * W},${Hh - Math.min(1, p / 0.5) * (Hh - 2) - 1}`).join(' ');
  const thresholdY = Hh - Math.min(1, P_TILDE_RESTART / 0.5) * (Hh - 2) - 1;
  return (
    <div className="flex flex-wrap items-center gap-x-4 gap-y-1 px-3 py-1.5 font-mono text-[0.72rem] tabular-nums">
      <span className="eyebrow">{title}</span>
      <span title="mean wall-clock time per step over the last 10 steps">{ms.toFixed(1)} ms/step</span>
      <span title="belief storage (grid cells × 8 bytes + the one-byte table; particles × 4 doubles)">{kb >= 1024 ? `${(kb / 1024).toFixed(1)} MB` : `${kb.toFixed(0)} kB`}</span>
      <span title="distance from the mode to the truth">|mode − truth| {err.toFixed(2)} m</span>
      <span title="entropy of the posterior in bits">H {H.toFixed(2)} b</span>
      <span className="flex items-center gap-1" title="p̃: per-beam average observation likelihood, the kidnap monitor of §9.1.4">
        <span className="text-fd-muted-foreground">p̃</span>
        <svg width={W} height={Hh} aria-hidden="true">
          <line x1={0} x2={W} y1={thresholdY} y2={thresholdY} stroke="var(--pr-truth)" strokeDasharray="2 2" strokeWidth={1} />
          {n > 1 ? <polyline points={pts} fill="none" stroke="var(--pr-measurement)" strokeWidth={1.4} /> : null}
        </svg>
        <span>{(pTilde[n - 1] ?? 0).toFixed(2)}</span>
      </span>
      {restarts !== undefined ? <span className="text-fd-muted-foreground">restarts {restarts}</span> : null}
    </div>
  );
}
