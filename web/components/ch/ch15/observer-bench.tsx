'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { ellipse2, matVec, type Mat, type Vec } from '@/lib/prob/linalg';
import { HALLWAY_1D } from '@/lib/sim/world';
import {
  cabs,
  covariancePredict,
  deadReckoning,
  eig2,
  errorMatrix,
  gainSchedule,
  kalmanGain,
  observerCovarianceUpdate,
  observerStep,
  type Lti,
  type RailSensor,
} from '@/lib/estimate';

/**
 * w15.1 — the Observer Bench.
 *
 * Choset's §8.2.6 robot on a rail, run two ways on the same noise: as a
 * fixed-gain Luenberger observer whose gain the reader sets, and as the Kalman
 * filter, whose gain R(k) is recomputed from P every step. The knob is scaled
 * so that g = 1 is the steady-state Kalman gain R∞ — the one constant gain the
 * schedule converges to — so handing the knob to the filter visibly moves it
 * toward 1. Everything the canvas draws comes from `lib/estimate`: the
 * estimates from `observerStep`/`kalmanGain`, the ellipses from
 * `covariancePredict`/`observerCovarianceUpdate`, the eigenvalues from `eig2`.
 */

const L = HALLWAY_1D.length;
const T = 0.5;
const V_TARGET = 0.6;

interface Params {
  gain: number;
  kalman: boolean;
  sensor: RailSensor;
  W: number;
  vScale: number;
}

interface Snapshot {
  x: Vec;
  P: Mat;
}

interface State {
  sys: Lti;
  rng: Rng;
  truth: Vec;
  /** Belief after the most recent operation. */
  bel: Snapshot;
  /** Belief before this operation — the ghost ellipse. */
  before: Snapshot;
  /** The prediction this step's update started from (null during a predict phase). */
  predicted: Snapshot | null;
  phase: 'predict' | 'update';
  /** Gain actually applied on the last update. */
  K: Mat;
  /** Steady-state Kalman gain for the current sensor/noise — the knob's unit. */
  Rinf: Mat;
  y: number | null;
  nu: number[];
  tracePick: number[];
  traceKalman: number[];
  /** A parallel Kalman covariance run, so observer mode can show what it is giving up. */
  Pkal: Mat;
  direction: 1 | -1;
  /** The parameters `sys` was built from, so a changed sensor or noise rebuilds it mid-run. */
  built: { sensor: RailSensor; W: number; vScale: number };
}

/** R∞ as the limit of the schedule: converges even when P itself does not (velocity sensor). */
function steadyGain(sys: Lti, P0: Mat): Mat {
  const sched = gainSchedule(sys, P0, 250);
  return sched[sched.length - 1];
}

export function ObserverBench() {
  const [params, setParams] = useState<Params>({ gain: 0.45, kalman: false, sensor: 'velocity', W: 0.5, vScale: 1 });

  const init = useCallback(
    (seed: number): State => {
      const { sys, bel } = deadReckoning({ T, sensor: params.sensor, W: params.W, processScale: params.vScale });
      const Rinf = steadyGain(sys, bel.P);
      const start = { x: [1.5, V_TARGET], P: bel.P };
      return {
        sys,
        rng: new Rng(seed),
        truth: [1.5 + 0.6, V_TARGET - 0.5],
        bel: start,
        before: start,
        predicted: null,
        phase: 'update',
        K: Rinf.map((r) => r.map((v) => v * params.gain)),
        Rinf,
        y: null,
        nu: [],
        tracePick: [],
        traceKalman: [],
        Pkal: bel.P,
        direction: 1,
        built: { sensor: params.sensor, W: params.W, vScale: params.vScale },
      };
    },
    // Only the seed matters for a reset; parameter changes are picked up live in `step`.
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  );

  const step = useCallback(
    (s0: State): State => {
      let s = s0;
      if (s.built.sensor !== params.sensor || s.built.W !== params.W || s.built.vScale !== params.vScale) {
        const { sys, bel } = deadReckoning({ T, sensor: params.sensor, W: params.W, processScale: params.vScale });
        s = { ...s, sys, Rinf: steadyGain(sys, bel.P), built: { sensor: params.sensor, W: params.W, vScale: params.vScale } };
      }
      const { sys, rng } = s;
      if (s.phase === 'update') {
        // ---- predict: the truth moves under a force the filter also knows ----
        let direction = s.direction;
        if (s.truth[0] > L - 1 && direction > 0) direction = -1;
        if (s.truth[0] < 1 && direction < 0) direction = 1;
        const u = [0.6 * (direction * V_TARGET - s.truth[1])];
        const noise: Vec = [rng.normal(0, Math.sqrt(sys.V[0][0])), rng.normal(0, Math.sqrt(sys.V[1][1]))];
        const truth = matVec(sys.F, s.truth).map((v, i) => v + sys.G[i][0] * u[0] + noise[i]);
        const xPred = matVec(sys.F, s.bel.x).map((v, i) => v + sys.G[i][0] * u[0]);
        const PPred = covariancePredict(s.bel.P, sys.F, sys.V);
        const Pkal = covariancePredict(s.Pkal, sys.F, sys.V);
        const predicted = { x: xPred, P: PPred };
        return { ...s, truth, before: s.bel, bel: predicted, predicted, phase: 'predict', y: null, Pkal, direction };
      }
      // ---- update: one noisy sensor reading, one gain ----
      const yTrue = sys.H[0][0] * s.truth[0] + sys.H[0][1] * s.truth[1];
      const y = yTrue + rng.normal(0, Math.sqrt(sys.W[0][0]));
      const Rk = kalmanGain(s.bel.P, sys.H, sys.W);
      const K = params.kalman ? Rk : s.Rinf.map((r) => r.map((v) => v * params.gain));
      // The observer step with K set to R(k) *is* the Kalman mean update (derivation 4).
      const { xhat, nu } = observerStep({ F: [[1, 0], [0, 1]], G: [[0], [0]], H: sys.H }, s.bel.x, K, [0], [y]);
      const P = observerCovarianceUpdate(s.bel.P, sys.H, sys.W, K);
      const Pkal = observerCovarianceUpdate(s.Pkal, sys.H, sys.W, kalmanGain(s.Pkal, sys.H, sys.W));
      return {
        ...s,
        before: s.bel,
        bel: { x: xhat, P },
        phase: 'update',
        K,
        y,
        nu: [...s.nu, nu[0]].slice(-48),
        tracePick: [...s.tracePick, P[0][0] + P[1][1]].slice(-120),
        traceKalman: [...s.traceKalman, Pkal[0][0] + Pkal[1][1]].slice(-120),
        Pkal,
      };
    },
    [params.kalman, params.gain, params.sensor, params.W, params.vScale],
  );

  const sim = useSimulation<State>({ init, step, fps: 4, initialSeed: 15 });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const s = sim.state;
      const { sys } = s;
      const isVelocity = sys.H[0][1] === 1;

      // ---------------- rail strip (top) ----------------
      const railY = 8.3;
      const railX0 = 0.6;
      const railX1 = 10.4;
      const toRail = (x: number) => railX0 + ((x / L) * (railX1 - railX0));
      ctx.save();
      ctx.strokeStyle = p.wall;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(sx(v, railX0), sy(v, railY));
      ctx.lineTo(sx(v, railX1), sy(v, railY));
      ctx.stroke();
      for (const d of HALLWAY_1D.doors) {
        ctx.strokeStyle = p.accent;
        ctx.beginPath();
        ctx.moveTo(sx(v, toRail(d)), sy(v, railY + 0.18));
        ctx.lineTo(sx(v, toRail(d)), sy(v, railY - 0.18));
        ctx.stroke();
      }
      // 95% position interval of the estimate, then the estimate and the truth.
      const sig = Math.sqrt(Math.max(s.bel.P[0][0], 0));
      const col = s.phase === 'predict' ? p.prediction : p.posterior;
      ctx.strokeStyle = col;
      ctx.lineWidth = 6;
      ctx.globalAlpha = 0.25;
      ctx.beginPath();
      ctx.moveTo(sx(v, toRail(Math.max(0, s.bel.x[0] - 1.96 * sig))), sy(v, railY));
      ctx.lineTo(sx(v, toRail(Math.min(L, s.bel.x[0] + 1.96 * sig))), sy(v, railY));
      ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.fillStyle = col;
      ctx.beginPath();
      ctx.arc(sx(v, toRail(s.bel.x[0])), sy(v, railY), 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = p.truth;
      ctx.beginPath();
      ctx.arc(sx(v, toRail(s.truth[0])), sy(v, railY + 0.32), 5, 0, Math.PI * 2);
      ctx.fill();
      label(ctx, 'truth', sx(v, toRail(s.truth[0])) + 8, sy(v, railY + 0.32), p.truth, { size: 10 });
      label(ctx, 'x̂', sx(v, toRail(s.bel.x[0])) + 8, sy(v, railY), col, { size: 10 });
      ctx.restore();

      // ---------------- phase plane (left) ----------------
      const pp = { x0: 0.6, x1: 7.6, y0: 0.9, y1: 7.4 };
      const cx = s.bel.x[0];
      const cv = s.bel.x[1];
      const spanX = 3.2;
      const spanV = 2.4;
      const px = (x: number) => pp.x0 + ((x - (cx - spanX)) / (2 * spanX)) * (pp.x1 - pp.x0);
      const pv = (vel: number) => pp.y0 + ((vel - (cv - spanV)) / (2 * spanV)) * (pp.y1 - pp.y0);
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.strokeRect(sx(v, pp.x0), sy(v, pp.y1), sx(v, pp.x1) - sx(v, pp.x0), sy(v, pp.y0) - sy(v, pp.y1));
      ctx.beginPath();
      ctx.rect(sx(v, pp.x0), sy(v, pp.y1), sx(v, pp.x1) - sx(v, pp.x0), sy(v, pp.y0) - sy(v, pp.y1));
      ctx.clip();
      label(ctx, 'position x_r →', sx(v, pp.x1) - 6, sy(v, pp.y0) + 12, p.ink, { size: 10, align: 'right' });
      label(ctx, '↑ velocity v_r', sx(v, pp.x0) + 6, sy(v, pp.y1) - 10, p.ink, { size: 10 });

      const drawEll = (snap: Snapshot, color: string, alpha: number, dashed = false) => {
        const e = ellipse2(snap.P, 2.447); // √χ²₂(0.95): the 95% ellipse, as in Figs. 8.5–8.7
        ctx.save();
        ctx.globalAlpha = alpha;
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.lineWidth = 1.75;
        if (dashed) ctx.setLineDash([4, 3]);
        ctx.beginPath();
        const steps = 72;
        for (let i = 0; i <= steps; i++) {
          const t = (i / steps) * 2 * Math.PI;
          const ex = e.rx * Math.cos(t);
          const ey = e.ry * Math.sin(t);
          const wx = snap.x[0] + ex * Math.cos(e.angle) - ey * Math.sin(e.angle);
          const wv = snap.x[1] + ex * Math.sin(e.angle) + ey * Math.cos(e.angle);
          const X = sx(v, px(wx));
          const Y = sy(v, pv(wv));
          if (i === 0) ctx.moveTo(X, Y);
          else ctx.lineTo(X, Y);
        }
        ctx.closePath();
        ctx.globalAlpha = alpha * 0.12;
        ctx.fill();
        ctx.globalAlpha = alpha;
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(sx(v, px(snap.x[0])), sy(v, pv(snap.x[1])), 3, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      };

      if (s.phase === 'predict') {
        drawEll(s.before, p.prior, 0.9);
        drawEll(s.bel, p.prediction, 1);
      } else {
        if (s.predicted) drawEll(s.predicted, p.prediction, 0.75);
        drawEll(s.bel, p.posterior, 1);
        // Ω = {x : H x = y}: a horizontal line for the velocity sensor, vertical for position.
        if (s.y !== null) {
          ctx.save();
          ctx.strokeStyle = p.measurement;
          ctx.lineWidth = 1.75;
          ctx.setLineDash([6, 4]);
          ctx.beginPath();
          if (isVelocity) {
            ctx.moveTo(sx(v, pp.x0), sy(v, pv(s.y)));
            ctx.lineTo(sx(v, pp.x1), sy(v, pv(s.y)));
          } else {
            ctx.moveTo(sx(v, px(s.y)), sy(v, pp.y0));
            ctx.lineTo(sx(v, px(s.y)), sy(v, pp.y1));
          }
          ctx.stroke();
          ctx.restore();
          label(ctx, 'Ω: Hx = y', isVelocity ? sx(v, pp.x0) + 6 : sx(v, px(s.y)) + 6, isVelocity ? sy(v, pv(s.y)) - 9 : sy(v, pp.y1) - 22, p.measurement, { size: 10 });
        }
      }
      // Truth as a gray cross.
      ctx.save();
      ctx.strokeStyle = p.truth;
      ctx.lineWidth = 1.5;
      const tx = sx(v, px(s.truth[0]));
      const ty = sy(v, pv(s.truth[1]));
      ctx.beginPath();
      ctx.moveTo(tx - 5, ty - 5);
      ctx.lineTo(tx + 5, ty + 5);
      ctx.moveTo(tx - 5, ty + 5);
      ctx.lineTo(tx + 5, ty - 5);
      ctx.stroke();
      ctx.restore();
      ctx.restore();

      // ---------------- unit disc (right, top) ----------------
      const disc = { cx: 11.6, cy: 5.9, r: 1.25 };
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(sx(v, disc.cx), sy(v, disc.cy), sx(v, disc.cx + disc.r) - sx(v, disc.cx), 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(sx(v, disc.cx - disc.r * 1.15), sy(v, disc.cy));
      ctx.lineTo(sx(v, disc.cx + disc.r * 1.15), sy(v, disc.cy));
      ctx.moveTo(sx(v, disc.cx), sy(v, disc.cy - disc.r * 1.15));
      ctx.lineTo(sx(v, disc.cx), sy(v, disc.cy + disc.r * 1.15));
      ctx.stroke();
      const E = errorMatrix(sys.F, sys.H, s.K);
      const eigs = eig2(E);
      const stable = eigs.every((z) => cabs(z) < 1 - 1e-9);
      for (const z of eigs) {
        ctx.fillStyle = cabs(z) < 1 - 1e-9 ? p.posterior : cabs(z) < 1 + 1e-9 ? p.cobstacle : p.goal;
        ctx.beginPath();
        ctx.arc(sx(v, disc.cx + z.re * disc.r), sy(v, disc.cy + z.im * disc.r), 4.5, 0, Math.PI * 2);
        ctx.fill();
      }
      label(ctx, 'eig (I − KH)F', sx(v, disc.cx), sy(v, disc.cy + disc.r) - 14, p.ink, { size: 10, align: 'center' });
      label(
        ctx,
        stable ? 'inside the unit disc: error decays' : eigs.some((z) => Math.abs(cabs(z) - 1) < 1e-9) ? '|λ| = 1: a direction never corrected' : 'outside: the observer diverges',
        sx(v, disc.cx),
        sy(v, disc.cy - disc.r) + 14,
        stable ? p.posterior : p.goal,
        { size: 9.5, align: 'center' },
      );
      ctx.restore();

      // ---------------- innovation strip (right, middle) ----------------
      const strip = { x0: 8.4, x1: 14.9, y0: 2.7, y1: 4.0 };
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.moveTo(sx(v, strip.x0), sy(v, (strip.y0 + strip.y1) / 2));
      ctx.lineTo(sx(v, strip.x1), sy(v, (strip.y0 + strip.y1) / 2));
      ctx.stroke();
      const sW = Math.sqrt(sys.W[0][0]);
      const scaleNu = (strip.y1 - strip.y0) / 2 / (3 * sW);
      const bw = (strip.x1 - strip.x0) / 48;
      ctx.fillStyle = p.measurement;
      s.nu.forEach((n, i) => {
        const h = Math.max(-1, Math.min(1, (n * scaleNu) / ((strip.y1 - strip.y0) / 2))) * ((strip.y1 - strip.y0) / 2);
        const X = sx(v, strip.x0 + i * bw);
        const Y0 = sy(v, (strip.y0 + strip.y1) / 2);
        const Y1 = sy(v, (strip.y0 + strip.y1) / 2 + h);
        ctx.fillRect(X, Math.min(Y0, Y1), Math.max(sx(v, strip.x0 + bw) - sx(v, strip.x0) - 1, 1), Math.abs(Y1 - Y0) || 1);
      });
      label(ctx, 'innovation ν = y − Hx̂⁻  (±3σ_w)', sx(v, strip.x0), sy(v, strip.y1) - 8, p.measurement, { size: 10 });
      ctx.restore();

      // ---------------- trace P sparkline (right, bottom) ----------------
      const spark = { x0: 8.4, x1: 14.9, y0: 0.6, y1: 2.1 };
      const all = [...s.tracePick, ...s.traceKalman];
      const maxT = Math.max(0.5, ...all);
      const plot = (series: number[], color: string, dashed: boolean) => {
        if (series.length < 2) return;
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        if (dashed) ctx.setLineDash([3, 3]);
        ctx.beginPath();
        series.forEach((t, i) => {
          const X = sx(v, spark.x0 + ((spark.x1 - spark.x0) * i) / 119);
          const Y = sy(v, spark.y0 + ((spark.y1 - spark.y0) * Math.min(t, maxT)) / maxT);
          if (i === 0) ctx.moveTo(X, Y);
          else ctx.lineTo(X, Y);
        });
        ctx.stroke();
        ctx.restore();
      };
      plot(s.traceKalman, p.posterior, true);
      plot(s.tracePick, params.kalman ? p.posterior : p.prediction, false);
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.moveTo(sx(v, spark.x0), sy(v, spark.y0));
      ctx.lineTo(sx(v, spark.x1), sy(v, spark.y0));
      ctx.stroke();
      ctx.restore();
      const lastT = s.tracePick[s.tracePick.length - 1];
      label(
        ctx,
        `tr P  (${params.kalman ? 'Kalman' : 'your gain'} solid, Kalman dashed)${lastT !== undefined ? `  now ${lastT.toFixed(2)}` : ''}`,
        sx(v, spark.x0),
        sy(v, spark.y1) - 8,
        p.ink,
        { size: 10 },
      );

      // Phase caption
      label(
        ctx,
        s.phase === 'predict' ? 'PREDICT   P⁻ = F P Fᵀ + V   (the ellipse grows)' : 'UPDATE   x̂⁺ = x̂⁻ + K ν   (the ellipse squishes toward Ω)',
        sx(v, 0.6),
        sy(v, 7.75),
        s.phase === 'predict' ? p.prediction : p.posterior,
        { size: 11, weight: 600 },
      );
    },
    [sim.state, params.kalman],
  );

  const readout = useMemo(() => {
    const s = sim.state;
    const K = s.K.map((r) => r[0]);
    const Rinf = s.Rinf.map((r) => r[0]);
    const n2 = Rinf[0] * Rinf[0] + Rinf[1] * Rinf[1];
    const knobEquivalent = n2 > 0 ? (K[0] * Rinf[0] + K[1] * Rinf[1]) / n2 : 0;
    const eigs = eig2(errorMatrix(s.sys.F, s.sys.H, s.K)).map(cabs);
    return { K, Rinf, knobEquivalent, eigs };
  }, [sim.state]);

  return (
    <WidgetFrame
      id="w15.1"
      title="Observer Bench"
      teaches="The Kalman gain is not a tuning constant. It is a trajectory R(k) — and the knob you were turning is what the filter computes."
      colorKey={['prior', 'prediction', 'measurement', 'posterior', 'truth']}
      wide
      caption={
        <>
          Choset&rsquo;s §8.2.6 robot on the hallway rail, state (x<sub>r</sub>, v<sub>r</sub>). Left: the 95%
          ellipse grows orange on predict and collapses purple on update toward the green line Ω of states
          consistent with the reading — Figures 8.5–8.7, live. The knob scales a fixed observer gain
          K = g·R<sub>∞</sub>; the dots on the unit disc are the eigenvalues of (I − KH)F. Turn g past the
          disc&rsquo;s edge and the observer diverges. Tick <em>let the filter choose</em>: the gain becomes
          R(k), the dashed and solid tr P curves merge, and the equivalent knob drifts to 1 on its own.
          With the velocity sensor one eigenvalue sits at exactly 1 whatever you do: position is never corrected.
        </>
      }
    >
      <SimCanvas
        world={{ minX: 0, minY: 0, maxX: 15.5, maxY: 9 }}
        draw={draw}
        deps={[sim.tick, sim.state, params.kalman]}
        aspect={15.5 / 9}
        padding={0}
        ariaLabel="A robot on a one-dimensional rail with its estimated position and uncertainty; beside it, the position-velocity phase plane with a prediction ellipse, an update ellipse and the measurement line; a unit disc with the observer's error eigenvalues; a strip of innovations; and a sparkline of the covariance trace."
      />

      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border text-center sm:grid-cols-4">
        <Stat label={params.kalman ? 'R(k) in use' : 'K in use'} value={`[${readout.K.map((k) => k.toFixed(3)).join(', ')}]`} />
        <Stat label="R∞ (Riccati limit)" value={`[${readout.Rinf.map((k) => k.toFixed(3)).join(', ')}]`} />
        <Stat label="equivalent knob g" value={readout.knobEquivalent.toFixed(2)} />
        <Stat label="|λ| of (I−KH)F" value={readout.eigs.map((m) => m.toFixed(3)).join(', ')} />
      </div>

      <ControlPanel columns={3}>
        <Slider
          label="Gain knob g  (K = g · R∞)"
          role="posterior"
          value={params.gain}
          min={0}
          max={6}
          step={0.05}
          onChange={(g) => setParams((q) => ({ ...q, gain: g }))}
          help="0 is pure dead reckoning; 1 is the steady-state Kalman gain; past about 3 (position sensor) or 5.6 (velocity sensor) an eigenvalue leaves the unit disc."
        />
        <Toggle
          label="Let the filter choose (Kalman)"
          role="posterior"
          checked={params.kalman}
          onChange={(k) => setParams((q) => ({ ...q, kalman: k }))}
        />
        <Toggle
          label={params.sensor === 'velocity' ? 'Sensor: velocity  H = [0 1]' : 'Sensor: position  H = [1 0]'}
          role="measurement"
          checked={params.sensor === 'position'}
          onChange={(pos) => setParams((q) => ({ ...q, sensor: pos ? 'position' : 'velocity' }))}
        />
        <Slider
          label="Measurement variance W"
          role="measurement"
          value={params.W}
          min={0.05}
          max={2}
          step={0.05}
          onChange={(w) => setParams((q) => ({ ...q, W: w }))}
        />
        <Slider
          label="Process noise scale (× Choset's V)"
          role="prediction"
          value={params.vScale}
          min={0.1}
          max={3}
          step={0.1}
          onChange={(k) => setParams((q) => ({ ...q, vScale: k }))}
        />
      </ControlPanel>

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

function Stat({ label: l, value }: { label: string; value: string }) {
  return (
    <div className="px-2 py-1.5">
      <div className="eyebrow">{l}</div>
      <div className="font-mono text-[0.8rem] tabular-nums">{value}</div>
    </div>
  );
}
