/**
 * Rusty, as the planning book sees it.
 *
 * The rover itself — chassis kinematics, encoders, slip, LiDAR — is the sister
 * book's and is ported unchanged in `lib/sim/rusty.ts`; nothing here
 * re-derives it. What this module adds is the *planner's* view: Rusty's
 * footprint is a disc of radius r = 0.11 m, so for collision purposes its
 * configuration is the disc center in R² (the heading is a configuration
 * variable for the *motion* model, but R(q) does not depend on it).
 */

import type { Pose2 } from '../geom/se2';
import type { Point2 } from '../sim/world';
import { RUSTY, type RobotParams } from '../sim/rusty';
import type { R2 } from '../collide/collision';

export {
  RUSTY,
  RUSTY_LIDAR,
  diffDriveSlipStep,
  wheelSpeeds,
  bodyTwist,
  pursuePoint,
  segments,
  raycastScan,
} from '../sim/rusty';
export type { RobotParams, RustyState, Twist, Script, DriveOutcome, Scan } from '../sim/rusty';
export { diffDriveStep } from '../sim/world';

/** R(q) for the rover: a closed disc. */
export interface Disc {
  center: Point2;
  radius: number;
}

/** Body radius r — 0.11 m, inherited from the sister book's TurtleBot-class chassis. */
export const RUSTY_RADIUS = RUSTY.bodyRadius;

export function rustyFootprint(pose: Pose2, p: RobotParams = RUSTY): Disc {
  return { center: { x: pose.x, y: pose.y }, radius: p.bodyRadius };
}

/** The planner's configuration for collision checks: drop the heading. */
export function rustyCfg(pose: Pose2): R2 {
  return { x: pose.x, y: pose.y };
}
