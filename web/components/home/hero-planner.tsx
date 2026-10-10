'use client';

/**
 * The landing-page hero: an RRT growing through the Apartment.
 *
 * It is Chapter 12's `Rrt` from the book's `sampling` library — the same
 * object the chapter's widgets drive — on Rusty as a disc in the Apartment,
 * drawn in the book's colors from the live palette. Seeded, so every visitor
 * sees the same tree; it restarts on a fresh seed a moment after reaching the
 * goal.
 */

import { SimCanvas } from '@/components/sim/sim-canvas';
import { useSimulation } from '@/lib/sim/use-simulation';
import { Rng } from '@/lib/prob/rng';
import { clear, drawPath, drawWorld, sx, sy } from '@/lib/sim/draw';
import { APARTMENT } from '@/lib/sim/world';
import { APARTMENT_GOAL, APARTMENT_START, Rrt, StraightLine, discLab } from '@/lib/sampling';

interface State {
  rrt: Rrt<number[]>;
  settled: number;
  seed: number;
}

const STEP = 0.35;
const CLEARANCE = 0.2;
const GOAL_R = 0.4;
const ITERS_PER_TICK = 6;
const MAX_NODES = 4000;

function init(seed: number): State {
  const lab = discLab(APARTMENT, CLEARANCE);
  const rrt = new Rrt<number[]>(
    lab.space,
    lab.free,
    [...APARTMENT_START],
    [...APARTMENT_GOAL],
    { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), eta: STEP, goalBias: 0.05, goalRadius: GOAL_R },
    new Rng(seed),
  );
  return { rrt, settled: 0, seed };
}

function step(s: State): State {
  if (s.rrt.solved) {
    s.settled += 1;
    // Hold the finished path for three seconds, then start over on a new seed derived from this one.
    if (s.settled > 90) return init(Math.floor(new Rng(s.seed).uniform(1, 1 << 30)));
    return s;
  }
  for (let k = 0; k < ITERS_PER_TICK && !s.rrt.solved && s.rrt.tree.size < MAX_NODES; k++) s.rrt.extendOnce();
  return s;
}

export function HeroPlanner() {
  const sim = useSimulation<State>({ init, step, fps: 30 });
  const s = sim.state;
  const tree = s.rrt.tree;
  const path = s.rrt.path();
  return (
    <div className="overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas
        world={APARTMENT.bounds}
        ariaLabel="A rapidly-exploring random tree growing through an apartment floorplan toward a goal"
        deps={[sim.tick]}
        draw={(ctx, v, p) => {
          clear(ctx, v, p);
          drawWorld(ctx, v, APARTMENT, p);
          ctx.save();
          ctx.lineWidth = 1;
          ctx.strokeStyle = p.graph;
          ctx.globalAlpha = 0.7;
          ctx.beginPath();
          for (let i = 1; i < tree.size; i++) {
            const n = tree.nodes[i];
            const q = tree.nodes[n.parent].q;
            ctx.moveTo(sx(v, q[0]), sy(v, q[1]));
            ctx.lineTo(sx(v, n.q[0]), sy(v, n.q[1]));
          }
          ctx.stroke();
          ctx.restore();
          if (path) drawPath(ctx, v, path.map((q) => ({ x: q[0], y: q[1] })), p.path, { lineWidth: 3 });
          for (const [pt, color] of [
            [APARTMENT_START, p.start],
            [APARTMENT_GOAL, p.goal],
          ] as const) {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(sx(v, pt[0]), sy(v, pt[1]), 6, 0, Math.PI * 2);
            ctx.fill();
          }
        }}
      />
    </div>
  );
}
