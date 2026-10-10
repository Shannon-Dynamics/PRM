'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import {
  clear,
  drawCovariance,
  drawGrid,
  drawPath,
  drawRobot,
  drawSegments,
  label,
  sx,
  sy,
  type Palette,
  type Viewport,
} from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { ellipse2, type Mat, type Vec } from '@/lib/prob/linalg';
import { APARTMENT, type Landmark } from '@/lib/sim/world';
import { pursuePoint } from '@/lib/sim/rusty';
import { poseNees, positionBlock } from '@/lib/filters/ekf-localization';
import {
  ChosetEkf,
  KalmanSlam,
  linearizedObservability,
  rangeBearingH,
  rangeOnlyH,
  unicycleStep,
  type SensorMode,
  type UnicycleInput,
} from '@/lib/estimate';

/**
 * w15.2 — the Observability Probe.
 *
 * Rusty drives a square in room A of the Apartment while Choset's §8.3.1 EKF
 * localizes it against known landmarks — or, with the map forgotten, §8.4.2's
 * Kalman SLAM rebuilds them. The headline control is how many landmarks the
 * sensor can see, with a range-only switch. The panel prints rank[H; HF; HF²]
 * of the pair linearized at the current estimate: with one range-only landmark
 * it is 2 and the ellipse stretches along the constant-range circle for as long
 * as you care to watch. More iterations do not mean more certainty; along a
 * direction the sensors cannot see, the optimal filter is optimally uncertain.
 */

const ROOM = { minX: -0.3, minY: -0.3, maxX: 4.3, maxY: 4.1 };
const SQUARE = [
  { x: 0.9, y: 0.9 },
  { x: 3.1, y: 0.9 },
  { x: 3.1, y: 2.9 },
  { x: 0.9, y: 2.9 },
];
const DT = 0.1;
const DEFAULT_LANDMARKS: Landmark[] = [
  { x: 0.35, y: 0.35, id: 0 },
  { x: 3.65, y: 0.35, id: 1 },
  { x: 0.35, y: 3.45, id: 2 },
];
const V: Mat = [
  [0.0015, 0, 0],
  [0, 0.0015, 0],
  [0, 0, 0.0006],
];

interface Params {
  visible: number;
  rangeOnly: boolean;
  slam: boolean;
  sigmaRange: number;
  sigmaBearing: number;
}

interface State {
  rng: Rng;
  truth: Vec;
  waypoint: number;
  ekf: ChosetEkf;
  slam: KalmanSlam | null;
  landmarks: Landmark[];
  truthPath: { x: number; y: number }[];
  estPath: { x: number; y: number }[];
  /** Covariance before the last update, for the orange ghost. */
  Ppred: Mat;
  xPred: Vec;
  lastU: UnicycleInput;
  seen: number[];
  nees: number[];
  trace: number[];
  built: { mode: SensorMode; visible: number; slam: boolean; sigmaRange: number; sigmaBearing: number };
}

const P0: Mat = [
  [0.04, 0, 0],
  [0, 0.04, 0],
  [0, 0, 0.01],
];

function buildFilters(truth: Vec, landmarks: Landmark[], params: Params): { ekf: ChosetEkf; slam: KalmanSlam | null } {
  const mode: SensorMode = params.rangeOnly ? 'range-only' : 'range-bearing';
  const ekf = new ChosetEkf(truth.slice(), P0, {
    landmarks,
    mode,
    V,
    sigmaRange: params.sigmaRange,
    sigmaBearing: params.sigmaBearing,
  });
  const slam =
    params.slam && !params.rangeOnly
      ? new KalmanSlam(truth.slice(), [[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]], {
          V,
          sigmaRange: params.sigmaRange,
          sigmaBearing: params.sigmaBearing,
        })
      : null;
  return { ekf, slam };
}

export function ObservabilityProbe() {
  const [params, setParams] = useState<Params>({ visible: 1, rangeOnly: true, slam: false, sigmaRange: 0.08, sigmaBearing: 0.04 });
  const dragging = useRef<number | null>(null);

  const init = useCallback(
    (seed: number): State => {
      const truth: Vec = [0.9, 0.9, 0];
      const landmarks = DEFAULT_LANDMARKS.map((l) => ({ ...l }));
      const { ekf, slam } = buildFilters(truth, landmarks, params);
      return {
        rng: new Rng(seed),
        truth,
        waypoint: 1,
        ekf,
        slam,
        landmarks,
        truthPath: [{ x: truth[0], y: truth[1] }],
        estPath: [{ x: truth[0], y: truth[1] }],
        Ppred: P0,
        xPred: truth.slice(),
        lastU: [0, 0],
        seen: [],
        nees: [],
        trace: [],
        built: { mode: params.rangeOnly ? 'range-only' : 'range-bearing', visible: params.visible, slam: params.slam, sigmaRange: params.sigmaRange, sigmaBearing: params.sigmaBearing },
      };
    },
    // Parameter changes are applied live inside `step`; only the seed resets.
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  );

  const step = useCallback(
    (s0: State): State => {
      let s = s0;
      const mode: SensorMode = params.rangeOnly ? 'range-only' : 'range-bearing';
      const changed =
        s.built.mode !== mode ||
        s.built.slam !== params.slam ||
        s.built.sigmaRange !== params.sigmaRange ||
        s.built.sigmaBearing !== params.sigmaBearing;
      if (changed) {
        // Rebuild the filters around the *current* estimate so a toggle never teleports the robot.
        const { ekf, slam } = buildFilters((s.slam ?? s.ekf).x.slice(0, 3), s.landmarks, params);
        s = { ...s, ekf, slam, built: { ...s.built, mode, slam: params.slam, sigmaRange: params.sigmaRange, sigmaBearing: params.sigmaBearing } };
      }
      const { rng } = s;

      // ---- the truth drives the square (closing the loop on truth is a documented cheat) ----
      const pose = { x: s.truth[0], y: s.truth[1], theta: s.truth[2] };
      let waypoint = s.waypoint;
      const target = SQUARE[waypoint];
      if (Math.hypot(target.x - pose.x, target.y - pose.y) < 0.12) waypoint = (waypoint + 1) % SQUARE.length;
      const tw = pursuePoint(pose, SQUARE[waypoint], { speed: 0.45, gain: 2.2, maxOmega: 1.6, turnFirst: 0.5 });
      const u: UnicycleInput = [tw.v * DT, tw.omega * DT];
      let truth = unicycleStep(s.truth, u);
      truth = [
        truth[0] + rng.normal(0, Math.sqrt(V[0][0])),
        truth[1] + rng.normal(0, Math.sqrt(V[1][1])),
        truth[2] + rng.normal(0, Math.sqrt(V[2][2])),
      ];

      // ---- predict ----
      const filter = s.slam ?? s.ekf;
      filter.predict(u);
      const xPred = filter.x.slice(0, 3);
      const Ppred = filter.P.slice(0, 3).map((r) => r.slice(0, 3));

      // ---- sense the visible landmarks and update ----
      const visible = s.landmarks.slice(0, params.visible);
      const seen: number[] = [];
      if (s.slam) {
        visible.forEach((lm, i) => {
          const h = rangeBearingH(truth, lm);
          const y = [h[0] + rng.normal(0, params.sigmaRange), h[1] + rng.normal(0, params.sigmaBearing)];
          s.slam!.observe(y, lm.id);
          seen.push(i);
        });
      } else {
        const ys = visible.map((lm) => {
          if (mode === 'range-only') return [rangeOnlyH(truth, lm)[0] + rng.normal(0, params.sigmaRange)];
          const h = rangeBearingH(truth, lm);
          return [h[0] + rng.normal(0, params.sigmaRange), h[1] + rng.normal(0, params.sigmaBearing)];
        });
        if (ys.length > 0) s.ekf.updateKnown(ys, visible.map((_, i) => i));
        visible.forEach((_, i) => seen.push(i));
      }

      const x = filter.x;
      const P = filter.P;
      const n = poseNees({ x: truth[0], y: truth[1], theta: truth[2] }, { mu: { x: x[0], y: x[1], theta: x[2] }, Sigma: P.slice(0, 3).map((r) => r.slice(0, 3)) });
      return {
        ...s,
        truth,
        waypoint,
        truthPath: [...s.truthPath, { x: truth[0], y: truth[1] }].slice(-600),
        estPath: [...s.estPath, { x: x[0], y: x[1] }].slice(-600),
        Ppred,
        xPred,
        lastU: u,
        seen,
        nees: [...s.nees, n].slice(-200),
        trace: [...s.trace, P[0][0] + P[1][1] + P[2][2]].slice(-200),
        built: { ...s.built, visible: params.visible },
      };
    },
    [params],
  );

  const sim = useSimulation<State>({ init, step, fps: 12, initialSeed: 7 });

  const obs = useMemo(() => {
    const s = sim.state;
    const mode: SensorMode = params.rangeOnly ? 'range-only' : 'range-bearing';
    return linearizedObservability(s.ekf.x, s.lastU, s.landmarks.slice(0, params.visible), mode);
  }, [sim.state, params.rangeOnly, params.visible]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      const s = sim.state;
      const filter = s.slam ?? s.ekf;
      // Room A's walls only: the probe lives in one room so the ellipse stays legible.
      drawSegments(
        ctx,
        v,
        APARTMENT.walls.filter((w) => Math.max(w.x1, w.x2) <= 4.01 && Math.max(w.y1, w.y2) <= 3.81),
        p.wall,
        2.5,
      );

      // Landmarks: visible ones solid, hidden ones hollow. Draggable.
      s.landmarks.forEach((lm, i) => {
        const vis = i < params.visible;
        ctx.save();
        ctx.strokeStyle = p.accent;
        ctx.fillStyle = p.accent;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(sx(v, lm.x), sy(v, lm.y), 6, 0, Math.PI * 2);
        if (vis) ctx.fill();
        else ctx.stroke();
        ctx.restore();
        label(ctx, `ℓ${i + 1}`, sx(v, lm.x) + 9, sy(v, lm.y) - 8, p.accent, { size: 10 });
      });

      // Sightings as green rays from the *true* pose (the sensor sits on the real robot).
      for (const i of s.seen) {
        const lm = s.landmarks[i];
        ctx.save();
        ctx.strokeStyle = p.measurement;
        ctx.globalAlpha = 0.6;
        ctx.lineWidth = 1.25;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(sx(v, s.truth[0]), sy(v, s.truth[1]));
        ctx.lineTo(sx(v, lm.x), sy(v, lm.y));
        ctx.stroke();
        ctx.restore();
        if (params.rangeOnly) {
          // The constant-range circle: the Ω of a single range sighting.
          const r = Math.hypot(s.truth[0] - lm.x, s.truth[1] - lm.y);
          ctx.save();
          ctx.strokeStyle = p.measurement;
          ctx.globalAlpha = 0.35;
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.arc(sx(v, lm.x), sy(v, lm.y), sx(v, lm.x + r) - sx(v, lm.x), 0, Math.PI * 2);
          ctx.stroke();
          ctx.restore();
        }
      }

      drawPath(ctx, v, s.truthPath, p.truth, { dashed: true, lineWidth: 1.5 });
      drawPath(ctx, v, s.estPath, p.posterior, { lineWidth: 1.75, alpha: 0.9 });

      // Prediction ghost, then the posterior ellipse.
      drawCovariance(ctx, v, [s.xPred[0], s.xPred[1]], ellipse2(positionBlock(s.Ppred), 2.447), p.prediction, { alpha: 0.6, fill: false });
      const x = filter.x;
      const Ppos = positionBlock(filter.P);
      drawCovariance(ctx, v, [x[0], x[1]], ellipse2(Ppos, 2.447), p.posterior);

      // SLAM map: estimated landmarks and their ellipses, in the prior color.
      if (s.slam) {
        for (let i = 0; i < s.slam.landmarkCount; i++) {
          const lm = s.slam.landmark(i);
          drawCovariance(ctx, v, [lm.x, lm.y], ellipse2(s.slam.landmarkCov(i), 2.447), p.prior, { alpha: 0.9 });
        }
      }

      // Blind direction(s): the (x, y) shadow of null(Q), drawn through the estimate.
      for (const b of obs.blind) {
        const len = Math.hypot(b[0], b[1]);
        if (len < 0.3) continue;
        ctx.save();
        ctx.strokeStyle = p.goal;
        ctx.lineWidth = 2;
        ctx.globalAlpha = 0.8;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(sx(v, x[0] - (b[0] / len) * 0.6), sy(v, x[1] - (b[1] / len) * 0.6));
        ctx.lineTo(sx(v, x[0] + (b[0] / len) * 0.6), sy(v, x[1] + (b[1] / len) * 0.6));
        ctx.stroke();
        ctx.restore();
      }

      drawRobot(ctx, v, { x: s.truth[0], y: s.truth[1], theta: s.truth[2] }, p.truth, 0.2, { filled: false });
      drawRobot(ctx, v, { x: x[0], y: x[1], theta: x[2] }, p.robot, 0.2);

      label(
        ctx,
        `rank[H; HF; HF²] = ${obs.rank}${obs.rank === 3 ? '  observable (locally)' : '  unobservable: ' + (3 - obs.rank) + ' blind direction' + (3 - obs.rank > 1 ? 's' : '')}`,
        sx(v, ROOM.minX) + 10,
        sy(v, ROOM.maxY) + 14,
        obs.rank === 3 ? p.start : p.goal,
        { size: 11, weight: 600 },
      );
    },
    [sim.state, obs, params.visible, params.rangeOnly],
  );

  const onPointer = useCallback(
    (world: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'down') {
        const s = sim.state;
        let best = -1;
        let bestD = 0.3;
        s.landmarks.forEach((lm, i) => {
          const d = Math.hypot(lm.x - world[0], lm.y - world[1]);
          if (d < bestD) {
            bestD = d;
            best = i;
          }
        });
        dragging.current = best >= 0 ? best : null;
      } else if (phase === 'move' && dragging.current !== null) {
        const i = dragging.current;
        sim.setState((s) => {
          // Mutate in place: the filters hold this same array, so the map they
          // localize against moves with the handle without a rebuild.
          const lm = s.landmarks[i];
          lm.x = Math.max(0.1, Math.min(3.9, world[0]));
          lm.y = Math.max(0.1, Math.min(3.7, world[1]));
          return { ...s };
        });
      } else if (phase === 'up') {
        dragging.current = null;
      }
    },
    [sim],
  );

  const stats = useMemo(() => {
    const s = sim.state;
    const last = s.nees[s.nees.length - 1] ?? 0;
    const mean = s.nees.length ? s.nees.reduce((a, b) => a + b, 0) / s.nees.length : 0;
    const Ppos = positionBlock((s.slam ?? s.ekf).P);
    const e = ellipse2(Ppos, 1);
    return { nees: last, meanNees: mean, major: e.rx, minor: e.ry, trace: s.trace[s.trace.length - 1] ?? 0, landmarks: s.slam?.landmarkCount ?? null };
  }, [sim.state]);

  return (
    <WidgetFrame
      id="w15.2"
      title="Observability Probe"
      teaches="More iterations do not mean more certainty. Along a direction the sensors cannot see, the optimal filter is optimally uncertain — forever."
      colorKey={['prior', 'prediction', 'measurement', 'posterior', 'truth', 'robot']}
      caption={
        <>
          Rusty drives a square in room A; the EKF of §8.3 localizes it against the teal landmarks (drag them).
          With <strong>one range-only landmark</strong> the rank of the linearized pair is 2: the posterior
          ellipse stretches along the green constant-range circle and the red dashed line marks the direction no
          sighting corrects. Switch to two landmarks and watch it collapse; put both landmarks and the robot on one
          line and watch it fail again. Range-and-bearing with one landmark is observable only while the robot
          moves. <em>Forget the map</em> hands the same sightings to §8.4.2&rsquo;s Kalman SLAM, which founds the
          landmarks (blue ellipses) from the first sighting and sharpens them with every pass.
        </>
      }
    >
      <SimCanvas
        world={ROOM}
        draw={draw}
        deps={[sim.tick, sim.state, obs, params.visible, params.rangeOnly]}
        aspect={4.6 / 4.4}
        padding={0}
        onPointer={onPointer}
        ariaLabel="A room of the Apartment with a robot driving a square. Its true path is dashed gray and its estimated path purple with a covariance ellipse; teal landmarks are the beacons the filter sees; a text line prints the rank of the observability matrix."
      />

      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border text-center sm:grid-cols-4">
        <Stat label="rank Q (linearized)" value={String(obs.rank)} tone={obs.rank === 3 ? 'ok' : 'bad'} />
        <Stat label="σ along major / minor" value={`${stats.major.toFixed(2)} / ${stats.minor.toFixed(2)} m`} />
        <Stat label="tr P (pose)" value={stats.trace.toFixed(4)} />
        <Stat label={stats.landmarks === null ? 'NEES (3 expected)' : `NEES · map size ${stats.landmarks}`} value={`${stats.nees.toFixed(1)} (mean ${stats.meanNees.toFixed(1)})`} />
      </div>

      <ControlPanel columns={3}>
        <Slider
          label="Visible landmarks"
          role="measurement"
          value={params.visible}
          min={1}
          max={3}
          step={1}
          onChange={(n) => setParams((q) => ({ ...q, visible: n }))}
          format={(n) => (n === 1 ? 'one' : n === 2 ? 'two' : 'all three')}
        />
        <Toggle
          label="Range-only sensor (§8.3.3)"
          role="measurement"
          checked={params.rangeOnly}
          onChange={(r) => setParams((q) => ({ ...q, rangeOnly: r, slam: r ? false : q.slam }))}
        />
        <Toggle
          label="Forget the map (Kalman SLAM, §8.4.2)"
          role="prior"
          checked={params.slam}
          onChange={(k) => setParams((q) => ({ ...q, slam: k, rangeOnly: k ? false : q.rangeOnly }))}
        />
        <Slider
          label="Range noise σ_ρ"
          role="measurement"
          value={params.sigmaRange}
          min={0.02}
          max={0.3}
          step={0.01}
          unit="m"
          onChange={(x) => setParams((q) => ({ ...q, sigmaRange: x }))}
        />
        <Slider
          label="Bearing noise σ_φ"
          role="measurement"
          value={params.sigmaBearing}
          min={0.01}
          max={0.2}
          step={0.01}
          unit="rad"
          onChange={(x) => setParams((q) => ({ ...q, sigmaBearing: x }))}
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

function Stat({ label: l, value, tone }: { label: string; value: string; tone?: 'ok' | 'bad' }) {
  return (
    <div className="px-2 py-1.5">
      <div className="eyebrow">{l}</div>
      <div
        className="font-mono text-[0.8rem] tabular-nums"
        style={tone ? { color: tone === 'ok' ? 'var(--rm-start)' : 'var(--rm-goal)' } : undefined}
      >
        {value}
      </div>
    </div>
  );
}
