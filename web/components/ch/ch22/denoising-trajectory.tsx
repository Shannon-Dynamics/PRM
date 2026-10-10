'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { DEFAULT_GUIDANCE, DIFF_K, DIFF_T, SLAB, denoise, loadModels, swingSide, trajectoryValid, type DenoiseRun } from '@/lib/learn';

/**
 * w22.4 — Denoising Trajectory.
 *
 * Thirty-two one-dimensional trajectories (a configuration per time step,
 * pinned to 0 at both ends) start as Gaussian noise and are denoised for K
 * steps by a small net trained on demonstrations that swing above or below a
 * slab obstacle. At the last step every trajectory goes through the hard
 * collision test and is drawn purple (clear) or red (through the slab). With
 * guidance on, each step also descends the gradient of a smooth surrogate cost;
 * it raises the valid fraction and does not make it 1.
 */

const COUNT = 32;
const ARENA = { minX: -0.6, minY: -1.45, maxX: DIFF_T - 1 + 0.6, maxY: 1.45 };

interface State {
  run: DenoiseRun;
  k: number;
}

function build(seed: number, guidance: boolean): State {
  const run = denoise(loadModels().denoiser, new Rng(seed), COUNT, { guidance: guidance ? DEFAULT_GUIDANCE : null });
  return { run, k: 0 };
}

export function DenoisingTrajectory() {
  const [guidance, setGuidance] = useState(false);
  const [scrub, setScrub] = useState<number | null>(null);
  const init = useCallback((seed: number) => build(seed, guidance), [guidance]);
  const stepFn = useCallback((s: State): State => (s.k >= DIFF_K ? s : { ...s, k: s.k + 1 }), []);
  const sim = useSimulation<State>({ init, step: stepFn, fps: 10, initialSeed: 4 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
    setScrub(null);
  }, [guidance]);

  const s = sim.state;
  const k = scrub ?? s.k;
  const trajs = s.run.steps[k];
  const final = k >= DIFF_K;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      // Time grid.
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let t = 0; t < DIFF_T; t++) {
        ctx.moveTo(sx(v, t), sy(v, ARENA.minY));
        ctx.lineTo(sx(v, t), sy(v, ARENA.maxY));
      }
      ctx.moveTo(sx(v, ARENA.minX), sy(v, 0));
      ctx.lineTo(sx(v, ARENA.maxX), sy(v, 0));
      ctx.stroke();
      ctx.restore();
      // The slab: |q| < halfWidth for t in [t0, t1].
      ctx.save();
      ctx.fillStyle = p.obstacle;
      ctx.globalAlpha = 0.75;
      const x0 = sx(v, SLAB.t0 - 0.5);
      const x1 = sx(v, SLAB.t1 + 0.5);
      ctx.fillRect(x0, sy(v, SLAB.halfWidth), x1 - x0, sy(v, -SLAB.halfWidth) - sy(v, SLAB.halfWidth));
      ctx.restore();
      // Trajectories.
      for (const tau of trajs) {
        const valid = trajectoryValid(tau);
        ctx.save();
        ctx.strokeStyle = final ? (valid ? p.path : p.goal) : p.graph;
        ctx.globalAlpha = final ? 0.9 : 0.35 + 0.4 * (k / DIFF_K);
        ctx.lineWidth = final ? 1.8 : 1.2;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        for (let t = 0; t < DIFF_T; t++) {
          const X = sx(v, t);
          const Y = sy(v, Math.max(ARENA.minY, Math.min(ARENA.maxY, tau[t])));
          if (t === 0) ctx.moveTo(X, Y);
          else ctx.lineTo(X, Y);
        }
        ctx.stroke();
        ctx.restore();
      }
      // Start and goal (pinned).
      ctx.save();
      ctx.fillStyle = p.start;
      ctx.beginPath();
      ctx.arc(sx(v, 0), sy(v, 0), 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = p.goal;
      ctx.beginPath();
      ctx.arc(sx(v, DIFF_T - 1), sy(v, 0), 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      label(ctx, `denoising step ${k} of ${DIFF_K}${guidance ? ' · guided' : ''}`, sx(v, ARENA.minX) + 6, sy(v, ARENA.maxY) + 12, p.ink, { size: 11, weight: 600 });
      if (final) {
        const nValid = trajs.filter(trajectoryValid).length;
        label(ctx, `${nValid} of ${COUNT} clear the slab`, sx(v, ARENA.maxX) - 6, sy(v, ARENA.maxY) + 12, p.path, { size: 11, weight: 700, align: 'right' });
      }
      label(ctx, 'time step t →', sx(v, ARENA.maxX) - 6, sy(v, ARENA.minY) - 10, p.ink, { size: 10, align: 'right' });
      label(ctx, '↑ q', sx(v, ARENA.minX) + 6, sy(v, 1.2), p.ink, { size: 10 });
      label(ctx, 'slab', sx(v, (SLAB.t0 + SLAB.t1) / 2), sy(v, 0), p.bg, { size: 10, align: 'center', weight: 700 });
    },
    [trajs, final, k, guidance],
  );

  const stats = useMemo(() => {
    const fin = s.run.steps[DIFF_K];
    const valid = fin.filter(trajectoryValid).length;
    const above = fin.filter((t) => swingSide(t) === 1).length;
    return { valid, above, below: COUNT - above };
  }, [s.run]);

  return (
    <WidgetFrame
      id="w22.4"
      title="Denoising Trajectory"
      teaches="Generative models produce plausible trajectories, not collision-free ones: the valid fraction is high and never one, so Chapter 2's checker runs on every sample."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path']}
      caption={
        <>
          A toy Diffuser: τ ∈ ℝ<sup>{DIFF_T}</sup> is a configuration per time step, start and goal pinned at 0, and the slate slab
          blocks |q| &lt; {SLAB.halfWidth} for t ∈ [{SLAB.t0}, {SLAB.t1}]. The demonstrations swung around it either way, so the
          denoiser learned both modes and reproduces both — Derivation 5&rsquo;s point about multimodality. At step {DIFF_K} the hard
          verifier colors each sample purple or red. Guidance adds −w<sub>k</sub>∇J per step for a smooth J (smoothness plus a
          soft slab penalty) and raises the valid fraction; it cannot make a soft penalty into a hard constraint, which is why the
          red ones remain and why the checker is not optional. Scrub the step; re-roll the seed.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.0} padding={0.2} draw={draw} deps={[k, trajs, guidance]} ariaLabel="Thirty-two one-dimensional trajectories denoising from noise around a slab obstacle; at the final step valid ones are purple and colliding ones red." />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={3}>
          <StatTile label="valid at step K" value={`${stats.valid} / ${COUNT}`} role="path" trend={Number(((100 * stats.valid) / COUNT).toFixed(0))} trendLabel="percent" />
          <StatTile label="swing above / below" value={`${stats.above} / ${stats.below}`} role="graph" />
          <StatTile label="guidance" value={guidance ? 'on' : 'off'} role="robot" trend={guidance ? DEFAULT_GUIDANCE.obstacle : 0} trendLabel="slab weight" />
        </Dashboard>
      </div>
      <ControlPanel columns={2} title="headline — denoising step">
        <Slider label="step k" role="graph" value={k} min={0} max={DIFF_K} step={1} onChange={(x) => setScrub(Math.round(x))} />
        <Toggle label="cost guidance −w_k ∇J" checked={guidance} onChange={setGuidance} role="robot" />
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={() => {
          setScrub(null);
          sim.stepOnce();
        }}
        onReset={() => {
          setScrub(null);
          sim.reset();
        }}
        onReseed={() => {
          setScrub(null);
          sim.reseed();
        }}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}
