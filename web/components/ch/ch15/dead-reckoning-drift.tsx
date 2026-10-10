'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawCovariance, drawGrid, drawPath, drawRobot, drawWorld, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { ellipse2, type Mat } from '@/lib/prob/linalg';
import type { Pose2 } from '@/lib/geom/se2';
import { APARTMENT, type Landmark } from '@/lib/sim/world';
import {
  RUSTY,
  diffDriveSlipStep,
  encoderTicks,
  odometryDelta,
  pursuePoint,
  type EncoderTicks,
  type RobotParams,
  type RustyState,
} from '@/lib/sim/rusty';
import { positionBlock } from '@/lib/filters/ekf-localization';
import { ChosetEkf, rangeBearingH, type UnicycleInput } from '@/lib/estimate';

/**
 * The chapter's hook — Fig. 8.1 on Rusty.
 *
 * Rusty drives a loop through the Apartment on encoders alone. The orange
 * executed trajectory is what the robot *believes* it did; the gray dashed line
 * is what it did. They part within a room. The belief is Choset's §8.3.1 EKF
 * run predict-only, so its covariance (orange ellipse) grows honestly with the
 * drift. Tick "landmark fixes" and a single range–bearing sighting of a known
 * corner snaps the belief back — the whole chapter in one toggle.
 */

const DT = 0.1;
const ROUTE = [
  { x: 1.0, y: 4.4 },
  { x: 11.2, y: 4.4 },
  { x: 9.95, y: 4.4 },
  { x: 9.95, y: 0.9 },
  { x: 11.2, y: 0.9 },
  { x: 9.95, y: 0.9 },
  { x: 9.95, y: 4.4 },
  { x: 2.05, y: 4.4 },
  { x: 2.05, y: 0.9 },
  { x: 0.9, y: 0.9 },
  { x: 2.05, y: 0.9 },
  { x: 2.05, y: 4.4 },
];
const SIGHT_RANGE = 2.6;
const SIGHT_EVERY = 12;
const V: Mat = [
  [0.0002, 0, 0],
  [0, 0.0002, 0],
  [0, 0, 0.00015],
];

interface Params {
  fixes: boolean;
  radiusBias: number;
}

interface State {
  rng: Rng;
  robot: RustyState;
  ticks: EncoderTicks;
  waypoint: number;
  ekf: ChosetEkf;
  truthPath: Pose2[];
  estPath: Pose2[];
  lastFix: { from: Pose2; to: Landmark } | null;
  fixCount: number;
  drift: number[];
}

export function DeadReckoningDrift() {
  const [params, setParams] = useState<Params>({ fixes: false, radiusBias: 0.003 });
  const landmarks = APARTMENT.landmarks ?? [];

  const init = useCallback(
    (seed: number): State => {
      const pose: Pose2 = { x: 1.0, y: 4.4, theta: 0 };
      const robot: RustyState = { pose, wheelAngles: [0, 0] };
      return {
        rng: new Rng(seed),
        robot,
        ticks: encoderTicks(robot.wheelAngles, RUSTY),
        waypoint: 1,
        ekf: new ChosetEkf([pose.x, pose.y, pose.theta], [[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]], {
          landmarks,
          mode: 'range-bearing',
          V,
          sigmaRange: 0.08,
          sigmaBearing: 0.03,
        }),
        truthPath: [pose],
        estPath: [pose],
        lastFix: null,
        fixCount: 0,
        drift: [],
      };
    },
    [landmarks],
  );

  const step = useCallback(
    (s: State, tick: number): State => {
      const { rng } = s;
      // Half of Rusty's nominal slip, so the systematic bias — the point of Fig. 8.1 — stays legible.
      const chassis: RobotParams = { ...RUSTY, slipStd: 0.01, radiusBiasRight: params.radiusBias };

      // Truth: the loop is closed on ground truth (a documented cheat) so the
      // route survives minutes of drift while the reader watches the estimate.
      let waypoint = s.waypoint;
      const target = ROUTE[waypoint];
      if (Math.hypot(target.x - s.robot.pose.x, target.y - s.robot.pose.y) < 0.2) waypoint = (waypoint + 1) % ROUTE.length;
      const tw = pursuePoint(s.robot.pose, ROUTE[waypoint], { speed: 0.6, gain: 2.0, maxOmega: 1.5, turnFirst: 0.6 });
      const out = diffDriveSlipStep(s.robot, tw, DT, APARTMENT, chassis, rng);
      const robot: RustyState = { pose: out.pose, wheelAngles: out.wheelAngles };

      // Odometry: what the encoders say happened, fed to the EKF as Choset's u = [Δs, Δθ].
      const ticks = encoderTicks(robot.wheelAngles, RUSTY);
      const tau = odometryDelta(s.ticks, ticks, RUSTY);
      const u: UnicycleInput = [tau[0], tau[2]];
      s.ekf.predict(u);

      // A sighting: the nearest landmark within range, every SIGHT_EVERY ticks.
      let lastFix = s.lastFix;
      let fixCount = s.fixCount;
      if (params.fixes && tick % SIGHT_EVERY === 0) {
        let best: { lm: Landmark; i: number; d: number } | null = null;
        landmarks.forEach((lm, i) => {
          const d = Math.hypot(lm.x - robot.pose.x, lm.y - robot.pose.y);
          if (d < SIGHT_RANGE && (!best || d < best.d)) best = { lm, i, d };
        });
        if (best) {
          const b: { lm: Landmark; i: number; d: number } = best;
          const h = rangeBearingH([robot.pose.x, robot.pose.y, robot.pose.theta], b.lm);
          const y = [h[0] + rng.normal(0, 0.08), h[1] + rng.normal(0, 0.03)];
          s.ekf.updateKnown([y], [b.i]);
          lastFix = { from: robot.pose, to: b.lm };
          fixCount++;
        } else {
          lastFix = null;
        }
      } else if (tick % SIGHT_EVERY === 0) {
        lastFix = null;
      }

      const est = s.ekf.pose();
      const drift = Math.hypot(est.x - robot.pose.x, est.y - robot.pose.y);
      return {
        ...s,
        robot,
        ticks,
        waypoint,
        truthPath: [...s.truthPath, robot.pose].slice(-1400),
        estPath: [...s.estPath, est].slice(-1400),
        lastFix,
        fixCount,
        drift: [...s.drift, drift].slice(-300),
      };
    },
    [params.fixes, params.radiusBias, landmarks],
  );

  const sim = useSimulation<State>({ init, step, fps: 20, initialSeed: 8 });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      drawWorld(ctx, v, APARTMENT, p);
      const s = sim.state;
      drawPath(ctx, v, s.truthPath, p.truth, { dashed: true, lineWidth: 1.5 });
      drawPath(ctx, v, s.estPath, p.robot, { lineWidth: 2 });
      if (s.lastFix) {
        ctx.save();
        ctx.strokeStyle = p.measurement;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(sx(v, s.lastFix.from.x), sy(v, s.lastFix.from.y));
        ctx.lineTo(sx(v, s.lastFix.to.x), sy(v, s.lastFix.to.y));
        ctx.stroke();
        ctx.restore();
      }
      const est = s.ekf.pose();
      drawCovariance(ctx, v, [est.x, est.y], ellipse2(positionBlock(s.ekf.P), 2.447), s.lastFix ? p.posterior : p.prediction, { alpha: 0.9 });
      drawRobot(ctx, v, s.robot.pose, p.truth, 0.24, { filled: false });
      drawRobot(ctx, v, est, p.robot, 0.24);
      const d = s.drift[s.drift.length - 1] ?? 0;
      label(ctx, `dead-reckoning error ${d.toFixed(2)} m   fixes ${s.fixCount}`, sx(v, 0.2), sy(v, 9) - 12, p.ink, { size: 11 });
    },
    [sim.state],
  );

  const stats = useMemo(() => {
    const s = sim.state;
    const d = s.drift[s.drift.length - 1] ?? 0;
    const P = positionBlock(s.ekf.P);
    return { drift: d, sigma: Math.sqrt(Math.max(P[0][0] + P[1][1], 0) / 2) };
  }, [sim.state]);

  return (
    <WidgetFrame
      id="f15.0"
      title="Dead-Reckoning Drift"
      teaches="Integrating odometry only ever loses information. Without an observer, the error grows without bound — and nothing in the integration can tell you so."
      colorKey={['robot', 'truth', 'prediction', 'measurement', 'posterior']}
      caption={
        <>
          Rusty laps the Apartment on encoders alone. The orange track is where the odometry says it is; the gray
          dashed track is where it is. A {(params.radiusBias * 100).toFixed(1)}% mismatch in the right wheel&rsquo;s
          radius (Borenstein&rsquo;s unequal diameters) bends every straight run into an arc, and the slip noise
          scatters on top. The orange ellipse is the EKF&rsquo;s own account of the error, grown predict-only.
          Tick <em>landmark fixes</em>: whenever a mapped corner is within {SIGHT_RANGE} m, one range–bearing
          sighting (green) collapses the ellipse to purple and drags the track back onto the truth.
        </>
      }
    >
      <SimCanvas
        world={APARTMENT.bounds}
        draw={draw}
        deps={[sim.tick, sim.state]}
        aspect={12.8 / 9.8}
        padding={0.4}
        ariaLabel="The Apartment floorplan with a robot driving a loop. Its true path is a gray dashed line; its dead-reckoned path is orange and gradually bends away. An ellipse around the estimate grows until a landmark sighting shrinks it."
      />
      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border text-center">
        <Stat label="|x̂ − x| position error" value={`${stats.drift.toFixed(2)} m`} />
        <Stat label="claimed σ (position)" value={`${stats.sigma.toFixed(2)} m`} />
      </div>
      <ControlPanel columns={2}>
        <Toggle label="Landmark fixes (one sighting at a time)" role="measurement" checked={params.fixes} onChange={(f) => setParams((q) => ({ ...q, fixes: f }))} />
        <Slider
          label="Right-wheel radius bias"
          role="robot"
          value={params.radiusBias}
          min={0}
          max={0.03}
          step={0.001}
          format={(x) => `${(x * 100).toFixed(1)}%`}
          onChange={(b) => setParams((q) => ({ ...q, radiusBias: b }))}
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
