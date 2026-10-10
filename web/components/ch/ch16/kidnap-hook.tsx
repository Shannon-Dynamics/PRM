'use client';

import { useCallback } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawCovariance, drawParticles, drawPath, drawRobot, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { ellipse2 } from '@/lib/prob/linalg';
import type { Pose2 } from '@/lib/geom/se2';
import { APARTMENT } from '@/lib/sim/world';
import { cellCenter } from '@/lib/search/grid';
import { kidnappedPair, type BeliefNavigator, type NavigatorSnapshot } from '@/lib/estimate';

/**
 * f16.0 — the chapter's hook: the kidnapped robot.
 *
 * Rusty is switched on in room B of the Apartment but *believes* it is in room A
 * — the position-tracking belief Chapter 15 would hand a planner, confident and
 * wrong. Left: that belief, a tight Gaussian that only dead-reckons, drives the
 * Part II planner Up through a doorway that exists only in room A and into
 * room B's wall; the odometry still reports the move, so the belief sails on
 * into the corridor while the chassis stays put. Right: a uniform particle
 * cloud over free space knows nothing, moves to where every hypothesis stays
 * cheap, and finds the room in a dozen scans.
 */

const REST_TICKS = 12;
const MAX_STEPS = 80;

interface State {
  seed: number;
  truth: Pose2;
  gaussian: BeliefNavigator;
  cloud: BeliefNavigator;
  gSnap: NavigatorSnapshot;
  cSnap: NavigatorSnapshot;
  rest: number;
}

function build(seed: number): State {
  const { truth, gaussian, cloud } = kidnappedPair(APARTMENT, seed);
  return { seed, truth, gaussian, cloud, gSnap: gaussian.snapshot(), cSnap: cloud.snapshot(), rest: 0 };
}

export function KidnapHook() {
  const init = useCallback((seed: number): State => build(seed), []);

  const step = useCallback((s: State): State => {
    const done = (s.cloud.reached || s.cloud.steps >= MAX_STEPS) && (s.gaussian.reached || s.gaussian.steps >= MAX_STEPS || s.gaussian.collisions >= 3);
    if (done) {
      if (s.rest < REST_TICKS) return { ...s, rest: s.rest + 1 };
      return build(s.seed + 17);
    }
    const gSnap = s.gaussian.collisions >= 3 || s.gaussian.steps >= MAX_STEPS ? s.gSnap : s.gaussian.step();
    const cSnap = s.cloud.steps >= MAX_STEPS ? s.cSnap : s.cloud.step();
    return { ...s, gSnap, cSnap };
  }, []);

  const sim = useSimulation<State>({ init, step, fps: 3, initialSeed: 1609 });

  const drawScene = useCallback((ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, nav: BeliefNavigator) => {
    clear(ctx, v, p);
    drawWorld(ctx, v, APARTMENT, p);
    const goal = cellCenter(nav.lattice, nav.goalCell);
    ctx.save();
    ctx.fillStyle = p.goal;
    ctx.beginPath();
    ctx.arc(sx(v, goal.x), sy(v, goal.y), Math.max(5, sl(v, 0.16)), 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
    const path = nav.pathFromMode();
    if (path.length > 1) {
      drawPath(ctx, v, path, p.bg, { lineWidth: 6 });
      drawPath(ctx, v, path, p.path, { lineWidth: 2.6 });
    }
    drawPath(ctx, v, [...nav.trail, nav.truth], p.robot, { lineWidth: 2 });
  }, []);

  const drawGaussian = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      const nav = sim.state.gaussian;
      drawScene(ctx, v, p, nav);
      const mean = nav.belief.pf.mean();
      const cov = nav.belief.pf.positionCovariance();
      drawCovariance(ctx, v, [mean.x, mean.y], ellipse2(cov, 2.447), p.prediction, { alpha: 0.95 });
      drawRobot(ctx, v, mean, p.prediction, 0.26, { filled: false });
      drawRobot(ctx, v, nav.truth, p.robot, 0.26);
      const msg = nav.collisions > 0 ? `bumped ${nav.collisions}× — the belief thinks it is in the corridor` : 'confident: σ ≈ 0.1 m around the wrong room';
      label(ctx, `Ch. 15 belief, predict-only  ·  ${msg}`, sx(v, 0.2), sy(v, 9) - 12, p.ink, { size: 11 });
    },
    [sim.state, drawScene],
  );

  const drawCloud = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      const nav = sim.state.cloud;
      drawScene(ctx, v, p, nav);
      drawParticles(ctx, v, nav.belief.pf.particles, p.posterior, { showHeading: false, maxRadius: 1.8 });
      drawRobot(ctx, v, nav.truth, p.robot, 0.26);
      const msg = nav.reached ? `reached the goal in ${nav.steps} steps, ${nav.collisions} bumps` : `H = ${nav.belief.entropyBits().toFixed(2)} bits after ${nav.steps} scans`;
      label(ctx, `uniform cloud, replanned every tick  ·  ${msg}`, sx(v, 0.2), sy(v, 9) - 12, p.ink, { size: 11 });
    },
    [sim.state, drawScene],
  );

  return (
    <WidgetFrame
      id="f16.0"
      title="The Kidnapped Robot"
      teaches="A confident Gaussian is worse than no belief at all when it is wrong. A belief that admits it knows nothing can still be planned on."
      colorKey={['prediction', 'posterior', 'path', 'goal', 'robot']}
      wide
      caption={
        <>
          Rusty (orange) is switched on in room B but was last seen in room A. <strong>Left:</strong> a tight, predict-only
          Gaussian (orange ellipse and hollow robot) says room A, so the Chapter 6 plan (purple) heads Up through A&rsquo;s doorway —
          which, in room B, is a wall. The wheels still report the move, so the belief drives on into the corridor while the chassis
          stays. <strong>Right:</strong> a uniform particle cloud (purple) over all free space, replanned every tick on the expected
          cost-to-go. It moves where every hypothesis stays cheap, the scans kill the rooms that do not match, and it arrives.
        </>
      }
    >
      <div className="grid grid-cols-1 border-b border-fd-border md:grid-cols-2 md:divide-x md:divide-fd-border">
        <SimCanvas
          world={APARTMENT.bounds}
          draw={drawGaussian}
          deps={[sim.tick, sim.state]}
          aspect={12.8 / 9.8}
          padding={0.4}
          ariaLabel="The Apartment with an orange robot in room B, an orange covariance ellipse in room A, and a purple planned path leaving room A through its door toward a red goal in the corridor. The robot bumps the wall above it."
        />
        <SimCanvas
          world={APARTMENT.bounds}
          draw={drawCloud}
          deps={[sim.tick, sim.state]}
          aspect={12.8 / 9.8}
          padding={0.4}
          ariaLabel="The same floorplan with a purple particle cloud spread over all rooms that gradually collapses onto the orange robot in room B as it moves and scans, then follows the corridor to the red goal."
        />
      </div>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
