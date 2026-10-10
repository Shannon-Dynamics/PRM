'use client';

import { useCallback, useMemo } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT, type Point2 } from '@/lib/sim/world';
import { GridGraph, cellCenter, freeCellNear, gridFromWorld, gridHeuristic } from '@/lib/search/grid';
import { bestFirst, PRIORITY_ASTAR } from '@/lib/search/best-first';
import { brushfire } from '@/lib/potential/brushfire';
import { RidgeGraph, featureTransform, gridGvdQuery, ridgeMaskFromFeatures } from '@/lib/roadmap/grid';
import { apartmentDistanceQuery, apartmentVisibility } from '@/lib/roadmap/examples';
import { minClearance, polylineLength } from '@/lib/roadmap/roadmap';
import { alongPath } from '@/lib/roadmap/retract';
import { marker } from '@/components/ch/ch07/shared';

/**
 * The chapter's hook: one Apartment query solved three ways. A* on the
 * Chapter 6 lattice, the visibility-graph path on disc-Rusty's C-obstacles
 * (hugging corners), and the GVD path (down the middle). Three Rustys drive
 * them at the same speed; the readout prints length and minimum clearance.
 */

const START: Point2 = { x: 1.0, y: 1.0 };
const GOAL: Point2 = { x: 10.6, y: 7.6 };
const RADIUS = 0.22;

interface Way {
  name: string;
  points: Point2[];
  length: number;
  clearance: number;
  color: 'graph' | 'path' | 'robot';
  dash: number[];
}

export function ThreeWaysHook() {
  const ways = useMemo<Way[]>(() => {
    const dq = apartmentDistanceQuery();
    const D = (p: Point2) => dq.nearest([p.x, p.y]).d;
    const out: Way[] = [];
    // 1. Lattice A* (Chapter 6): 0.25 m cells, inflated by the radius, octile heuristic with √2 diagonals.
    {
      const g = gridFromWorld(APARTMENT, 0.25, RADIUS);
      const graph = new GridGraph(g, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
      const s = freeCellNear(g, START.x, START.y);
      const t = freeCellNear(g, GOAL.x, GOAL.y);
      const tr = bestFirst(graph, s, t, gridHeuristic(g, t, 'octile', Math.SQRT2), PRIORITY_ASTAR);
      const pts = [START, ...tr.path.map((c) => cellCenter(g, c)), GOAL];
      out.push({ name: 'lattice A* (Ch. 6)', points: pts, length: polylineLength(pts), clearance: minClearance(pts, D), color: 'robot', dash: [2, 4] });
    }
    // 2. Visibility graph on the inflated walls.
    {
      const vs = apartmentVisibility(START, GOAL, RADIUS);
      const rm = vs.graph();
      const s = vs.nodes.find((n) => n.name === 'start')!.id;
      const t = vs.nodes.find((n) => n.name === 'goal')!.id;
      const p = rm.shortestPath(s, t);
      const pts = p ? p.points : [START, GOAL];
      out.push({ name: 'visibility graph', points: pts, length: polylineLength(pts), clearance: minClearance(pts, D), color: 'path', dash: [] });
    }
    // 3. The GVD, from the 0.1 m grid.
    {
      const g = gridFromWorld(APARTMENT, 0.1, 0.051);
      const dist = brushfire(g, 8);
      const ridge = new RidgeGraph(dist, ridgeMaskFromFeatures(g, featureTransform(g)));
      const q = gridGvdQuery(dist, ridge, freeCellNear(g, START.x, START.y), freeCellNear(g, GOAL.x, GOAL.y));
      const pts = q ? [START, ...q.points, GOAL] : [START, GOAL];
      out.push({ name: 'GVD', points: pts, length: polylineLength(pts), clearance: minClearance(pts, D, 2), color: 'graph', dash: [6, 3] });
    }
    return out;
  }, []);

  const maxLen = Math.max(...ways.map((w) => w.length));
  const sim = useSimulation<{ s: number }>({ init: () => ({ s: 0 }), step: (st) => ({ s: st.s + 0.06 > maxLen + 1.5 ? 0 : st.s + 0.06 }), fps: 30, initialSeed: 80 });
  const s = sim.state.s;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      for (const w of ways) {
        ctx.save();
        ctx.strokeStyle = p[w.color];
        ctx.lineWidth = 2;
        ctx.globalAlpha = 0.85;
        ctx.setLineDash(w.dash);
        ctx.beginPath();
        w.points.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
        ctx.stroke();
        ctx.restore();
      }
      marker(ctx, sx(v, START.x), sy(v, START.y), 7, p.start, p.bg);
      marker(ctx, sx(v, GOAL.x), sy(v, GOAL.y), 7, p.goal, p.bg);
      ways.forEach((w, k) => {
        const q = alongPath(w.points, Math.min(1, s / w.length));
        ctx.save();
        ctx.fillStyle = p[w.color];
        ctx.beginPath();
        ctx.arc(sx(v, q.x), sy(v, q.y), sl(v, RADIUS), 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
        label(ctx, `${w.name}: ${w.length.toFixed(2)} m, min clearance ${w.clearance.toFixed(2)} m`, 10, 14 + 14 * k, p[w.color], { size: 10.5, weight: 600 });
      });
    },
    [ways, s],
  );

  return (
    <div className="not-prose overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas world={APARTMENT.bounds} draw={draw} deps={[sim.tick]} aspect={12 / 9} padding={0.3} ariaLabel="The Apartment with three paths from a green start in room A to a red goal in the bedroom: an orange lattice path, a purple corner-hugging visibility path and a blue mid-corridor Voronoi path, each driven by a disc robot." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} tick={sim.tick} />
    </div>
  );
}
