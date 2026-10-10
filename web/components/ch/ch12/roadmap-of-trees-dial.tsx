'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import { APARTMENT_GOAL, APARTMENT_START, Srt, StraightLine, discLab, paintDot, tint, type DiscLab } from '@/lib/sampling';

/**
 * w12.3 — Roadmap-of-Trees Dial.
 *
 * Choset's SRT (§7.3, Algorithm 14) on the Apartment: a PRM whose nodes are
 * trees. The dial is nodes per tree. At (1 node, 1 close pair, 0 merge
 * iterations) the picture is w11.1's roadmap; with no roadmap trees at all it is
 * w12.1's single bidirectional tree; in between, blobs joined by close-pair
 * edges (solid) and merges (dashed) — Choset's figure 7.16. The build runs at
 * reset and the animation reveals it: trees, then edges, then the query's path.
 */

interface State {
  lab: DiscLab;
  srt: Srt<number[]>;
  path: number[][] | null;
  built: boolean;
  reveal: number;
}

const START_TREE_FRAMES = 1;

export function RoadmapOfTreesDial() {
  const [nodesPerTree, setNodesPerTree] = useState(25);
  const [trees, setTrees] = useState(14);
  const [closePairs, setClosePairs] = useState(3);
  const [mergeIters, setMergeIters] = useState(60);

  const init = useCallback(
    (seed: number): State => {
      const lab = discLab();
      const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
      const srt = new Srt<number[]>(lab.space, lab.free, { steer, nodesPerTree, closePairs, mergeIters, k: 3, r: 1, eta: 0.5 }, new Rng(seed));
      srt.addTrees(trees);
      srt.connectAll();
      const path = srt.query([...APARTMENT_START], [...APARTMENT_GOAL]);
      return { lab, srt, path, built: true, reveal: 0 };
    },
    [nodesPerTree, trees, closePairs, mergeIters],
  );

  const stepFn = useCallback((s: State): State => {
    const total = s.srt.trees.length + s.srt.edges.length + 30;
    if (s.reveal < total) s.reveal += 1;
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 20, initialSeed: 12 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [nodesPerTree, trees, closePairs, mergeIters]);

  const s = sim.state;
  const isPrm = nodesPerTree === 1 && closePairs === 1 && mergeIters === 0;
  const isSingle = trees === 0;
  const treesShown = Math.min(s.srt.trees.length, s.reveal * START_TREE_FRAMES);
  const edgesShown = Math.max(0, Math.min(s.srt.edges.length, s.reveal - s.srt.trees.length));
  const pathShown = s.reveal >= s.srt.trees.length + s.srt.edges.length + 10;
  let totalNodes = 0;
  for (const t of s.srt.trees) totalNodes += t.size;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, APARTMENT, p);
      const colorOf = (i: number) => tint(p.graph, ((i * 7) % 5) * 11, p.accent);
      s.srt.trees.forEach((t, ti) => {
        if (ti >= treesShown) return;
        const c = colorOf(ti);
        ctx.save();
        ctx.strokeStyle = c;
        ctx.lineWidth = 1;
        ctx.globalAlpha = 0.8;
        ctx.beginPath();
        for (let i = 1; i < t.size; i++) {
          const n = t.nodes[i];
          const q = t.nodes[n.parent].q;
          ctx.moveTo(sx(v, q[0]), sy(v, q[1]));
          ctx.lineTo(sx(v, n.q[0]), sy(v, n.q[1]));
        }
        ctx.stroke();
        ctx.restore();
        for (const n of t.nodes) paintDot(ctx, sx(v, n.q[0]), sy(v, n.q[1]), t.size === 1 ? 3 : 1.6, c, 0.9);
        // The root, the tree's representative.
        paintDot(ctx, sx(v, t.root[0]), sy(v, t.root[1]), 4, p.bg);
        paintDot(ctx, sx(v, t.root[0]), sy(v, t.root[1]), 3, c);
      });
      s.srt.edges.forEach((e, k) => {
        if (k >= edgesShown) return;
        const a = s.srt.trees[e.i].nodes[e.qi].q;
        const b = s.srt.trees[e.j].nodes[e.qj].q;
        ctx.save();
        ctx.strokeStyle = p.ink;
        ctx.lineWidth = e.merged ? 1.5 : 2;
        ctx.globalAlpha = 0.7;
        if (e.merged) ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.moveTo(sx(v, a[0]), sy(v, a[1]));
        ctx.lineTo(sx(v, b[0]), sy(v, b[1]));
        ctx.stroke();
        ctx.restore();
      });
      if (pathShown && s.path) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        s.path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q[0]), sy(v, q[1])) : ctx.lineTo(sx(v, q[0]), sy(v, q[1]))));
        ctx.stroke();
        ctx.restore();
      }
      paintDot(ctx, sx(v, APARTMENT_START[0]), sy(v, APARTMENT_START[1]), 7, p.start);
      paintDot(ctx, sx(v, APARTMENT_GOAL[0]), sy(v, APARTMENT_GOAL[1]), 7, p.goal);
      const badge = isPrm ? 'this is BASIC PRM (Alg. 6)' : isSingle ? 'this is a bidirectional RRT (Alg. 13)' : `SRT: ${s.srt.trees.length} trees, ${s.srt.edges.length} roadmap edges`;
      label(ctx, badge, sx(v, 0.2), sy(v, 8.7), p.ink, { size: 11, weight: 700 });
      if (pathShown) label(ctx, s.path ? `query answered: ${s.path.length} waypoints` : 'query failed: start and goal in different components', sx(v, 11.8), sy(v, 8.7), s.path ? p.path : p.goal, { size: 11, weight: 600, align: 'right' });
    },
    [s, treesShown, edgesShown, pathShown, isPrm, isSingle],
  );

  return (
    <WidgetFrame
      id="w12.3"
      title="Roadmap-of-Trees Dial"
      teaches="PRM and RRT are not different algorithms but two corners of one box: SRT with one node per tree is PRM, SRT with no roadmap trees is a bidirectional RRT, and everything between is a roadmap whose nodes are trees."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Choset&rsquo;s Sampling-Based Roadmap of Trees. Every tint is one tree, grown by RRT from a
          sampled root (the larger dot). Trees are joined like PRM nodes: a few <em>close pairs</em>{' '}
          are tried with the local planner (solid edges), and if none succeeds a short bidirectional
          merge is attempted (dashed). The query grows two more trees from the start and goal and
          walks the roadmap of trees. Turn nodes-per-tree down to 1, close pairs to 1 and merges to 0:
          the picture is Chapter 11&rsquo;s roadmap. Turn roadmap trees to 0: it is a single
          bidirectional RRT. The planners of this chapter are settings of one dial.
        </>
      }
    >
      <SimCanvas world={APARTMENT.bounds} aspect={12 / 9} draw={draw} deps={[sim.tick, s]} ariaLabel="A roadmap whose nodes are small trees, drawn in distinct tints across the apartment, joined by solid close-pair edges and dashed merge edges, with the query path in purple." />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="trees (incl. query)" value={s.srt.trees.length} precision={0} role="graph" />
          <StatTile label="configurations" value={totalNodes} precision={0} role="graph" />
          <StatTile label="roadmap edges" value={`${s.srt.edges.length} (${s.srt.edges.filter((e) => e.merged).length} merged)`} />
          <StatTile label="collision checks" value={s.lab.free.checks} precision={0} role="obstacle" />
        </Dashboard>
      </div>
      <ControlPanel columns={2} title="headline — nodes per tree">
        <Slider label="nodes per tree" role="graph" value={nodesPerTree} min={1} max={200} step={1} onChange={setNodesPerTree} format={(x) => (x === 1 ? '1 — PRM corner' : x.toFixed(0))} />
        <Slider label="roadmap trees" value={trees} min={0} max={30} step={1} onChange={setTrees} format={(x) => (x === 0 ? '0 — RRT corner' : x.toFixed(0))} />
        <Slider label="close pairs per edge" value={closePairs} min={0} max={6} step={1} onChange={setClosePairs} />
        <Slider label="merge iterations" value={mergeIters} min={0} max={150} step={10} onChange={setMergeIters} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
