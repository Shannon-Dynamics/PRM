'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { cellIJ } from '@/lib/search/grid';
import { paintDot, rectMapper, tint } from '@/lib/sampling';
import { Cbs, at, countConflicts, jointAStar, makespan, roomInstance, type CtNode, type JointResult, type MapfInstance } from '@/lib/multi';

/**
 * w14.4 — Constraint Tree.
 *
 * CBS on a small room with 2–5 Rustys. Left: the grid, the agents replaying
 * the paths of the constraint-tree node being looked at, the first conflict
 * flashing red at its time. Right: the constraint tree, one node per
 * constraint set, labelled with SIC; children add one constraint each.
 * The readout sets the tree's size against the joint-state count a coupled
 * A* generates on the same instance — run for ≤ 3 agents, quoted as the
 * branching factor 5ᵐ beyond that.
 */

const ARENA = { minX: 0, minY: 0, maxX: 2.1, maxY: 1 };
const GRID = { x0: 0, y0: 0.1, x1: 1.0, y1: 0.95 };
const TREE = { x0: 1.1, y0: 0, x1: 2.1, y1: 1 };
const TICKS_PER_EXPANSION = 24;

interface State {
  inst: MapfInstance;
  cbs: Cbs;
  joint: JointResult | null;
  /** The node whose paths the grid shows. */
  view: number;
  t: number;
  countdown: number;
}

export function ConstraintTree() {
  const [agents, setAgents] = useState(3);
  const [showJoint, setShowJoint] = useState(true);

  const init = useCallback((): State => {
    const inst = roomInstance(agents);
    const cbs = new Cbs(inst);
    const joint = agents <= 3 ? jointAStar(inst, 300000) : null;
    return { inst, cbs, joint, view: 0, t: 0, countdown: TICKS_PER_EXPANSION };
  }, [agents]);

  const expand = (s: State) => {
    const n = s.cbs.step();
    if (n) {
      s.view = n.id;
      s.t = 0;
    }
  };

  const stepFn = useCallback((s: State): State => {
    const node = s.cbs.nodes[s.view];
    if (node) {
      const T = makespan(node.paths);
      s.t = Math.min(T + 1, s.t + 0.25);
    }
    if (!s.cbs.solved && !s.cbs.exhausted) {
      s.countdown--;
      if (s.countdown <= 0) {
        expand(s);
        s.countdown = TICKS_PER_EXPANSION;
      }
    } else if (node && s.t >= makespan(node.paths) + 1) s.t = 0;
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 24, initialSeed: 14 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [agents]);

  const s = sim.state;
  const node: CtNode | undefined = s.cbs.nodes[s.view];

  const stepOnce = () => {
    sim.pause();
    sim.setState((st) => {
      if (!st.cbs.solved && !st.cbs.exhausted) expand(st);
      st.countdown = TICKS_PER_EXPANSION;
      return { ...st };
    });
  };

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const g = s.inst.grid;
      const m = rectMapper(v, GRID, { minX: 0, minY: 0, maxX: g.nx, maxY: g.ny });
      const cw = m.S(1);
      for (let j = 0; j < g.ny; j++) {
        for (let i = 0; i < g.nx; i++) {
          ctx.save();
          ctx.fillStyle = g.occupied[j * g.nx + i] ? p.obstacle : p.free;
          ctx.strokeStyle = p.grid;
          ctx.fillRect(m.X(i), m.Y(j + 1), cw, cw);
          ctx.strokeRect(m.X(i), m.Y(j + 1), cw, cw);
          ctx.restore();
        }
      }
      const center = (cell: number): [number, number] => {
        const [i, j] = cellIJ(g, cell);
        return [m.X(i + 0.5), m.Y(j + 0.5)];
      };
      s.inst.starts.forEach((c, a) => {
        const [x, y] = center(c);
        paintDot(ctx, x, y, 5, p.start, 0.9);
        label(ctx, `${a + 1}`, x, y - 10, p.start, { size: 9, align: 'center', weight: 700 });
      });
      s.inst.goals.forEach((c, a) => {
        const [x, y] = center(c);
        paintDot(ctx, x, y, 5, p.goal, 0.9);
        label(ctx, `${a + 1}`, x, y - 10, p.goal, { size: 9, align: 'center', weight: 700 });
      });
      if (node) {
        node.paths.forEach((path, a) => {
          ctx.save();
          ctx.strokeStyle = node.id === s.cbs.solution?.id ? p.path : p.graph;
          ctx.lineWidth = 2;
          ctx.globalAlpha = 0.55;
          if (a % 2 === 1) ctx.setLineDash([5, 4]);
          ctx.beginPath();
          path.forEach((c, i) => {
            const [x, y] = center(c);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          });
          ctx.stroke();
          ctx.restore();
        });
        const ti = Math.floor(s.t);
        const f = s.t - ti;
        node.paths.forEach((path, a) => {
          const [x0, y0] = center(at(path, ti));
          const [x1, y1] = center(at(path, ti + 1));
          const x = x0 + f * (x1 - x0);
          const y = y0 + f * (y1 - y0);
          ctx.save();
          ctx.fillStyle = p.robot;
          ctx.strokeStyle = p.robot;
          ctx.lineWidth = 2;
          ctx.globalAlpha = 0.9;
          ctx.beginPath();
          ctx.arc(x, y, cw * 0.34, 0, Math.PI * 2);
          if (a % 2 === 0) ctx.fill();
          else ctx.stroke();
          ctx.restore();
          label(ctx, `${a + 1}`, x, y, a % 2 === 0 ? p.bg : p.robot, { size: 10, align: 'center', weight: 700 });
        });
        if (node.conflict && Math.abs(s.t - node.conflict.t) < 0.6) {
          const [x, y] = center(node.conflict.cell);
          ctx.save();
          ctx.strokeStyle = p.goal;
          ctx.lineWidth = 3;
          ctx.globalAlpha = 1 - Math.abs(s.t - node.conflict.t) / 0.6;
          ctx.beginPath();
          ctx.arc(x, y, cw * 0.6, 0, Math.PI * 2);
          ctx.stroke();
          ctx.restore();
        }
        const conflictText = node.conflict ? `${node.conflict.edge ? 'edge' : 'vertex'} conflict: agents ${node.conflict.a + 1} and ${node.conflict.b + 1} at t = ${node.conflict.t}` : node.expanded ? 'conflict-free' : 'not yet expanded';
        label(ctx, `node ${node.id}: SIC = ${node.cost}, ${node.constraints.length} constraint${node.constraints.length === 1 ? '' : 's'}; ${conflictText}`, m.X(0), m.Y(g.ny) - 10, node.conflict ? p.goal : node.expanded ? p.path : p.ink, { size: 10, weight: 700 });
        label(ctx, `t = ${s.t.toFixed(1)}`, m.X(g.nx), m.Y(0) + 11, p.ink, { size: 10, align: 'right' });
      }

      // --- constraint tree ---
      const tm = rectMapper(v, TREE, { minX: 0, minY: 0, maxX: 1, maxY: 1 });
      const nodes = s.cbs.nodes;
      const depth = new Map<number, number>();
      const kidsByDepth = new Map<number, number[]>();
      for (const n of nodes) {
        const d = n.parent === null ? 0 : (depth.get(n.parent) ?? 0) + 1;
        depth.set(n.id, d);
        if (!kidsByDepth.has(d)) kidsByDepth.set(d, []);
        kidsByDepth.get(d)!.push(n.id);
      }
      const maxDepth = Math.max(1, ...[...kidsByDepth.keys()]);
      const pos = new Map<number, [number, number]>();
      for (const [d, ids] of kidsByDepth) ids.forEach((id, k) => pos.set(id, [tm.X((k + 1) / (ids.length + 1)), tm.Y(1 - 0.08 - (0.84 * d) / Math.max(maxDepth, 1))]));
      label(ctx, 'constraint tree — best-first on SIC; each child adds one constraint', tm.X(0.5), tm.Y(1) - 6, p.ink, { size: 9.5, align: 'center', weight: 600 });
      for (const n of nodes) {
        if (n.parent === null) continue;
        const [x0, y0] = pos.get(n.parent)!;
        const [x1, y1] = pos.get(n.id)!;
        ctx.save();
        ctx.strokeStyle = tint(p.ink, 60, p.bg);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
        ctx.restore();
      }
      const next = s.cbs.peek();
      for (const n of nodes) {
        const [x, y] = pos.get(n.id)!;
        const isSol = s.cbs.solution?.id === n.id;
        const color = isSol ? p.path : n.expanded ? p.graph : n.id === next?.id ? p.robot : tint(p.ink, 40, p.bg);
        paintDot(ctx, x, y, n.id === s.view ? 9 : 7, color, 1);
        label(ctx, `${n.cost}`, x, y, p.bg, { size: 8.5, align: 'center', weight: 700 });
        if (n.added) {
          const [ci, cj] = cellIJ(g, n.added.cell);
          label(ctx, `a${n.added.agent + 1}${n.added.from !== undefined ? ' edge→' : ' ∉ '}(${ci},${cj})@${n.added.t}`, x, y + 13, p.ink, { size: 7.5, align: 'center' });
        }
      }
      const footer = s.cbs.solved ? `solved: SIC ${s.cbs.solution!.cost} after ${s.cbs.expansions} expansions, ${nodes.length} CT nodes` : s.cbs.exhausted ? 'exhausted' : `${s.cbs.expansions} expansions, ${nodes.length} nodes — next: node ${next?.id ?? '—'}`;
      label(ctx, footer, tm.X(0.5), tm.Y(0) + 8, s.cbs.solved ? p.path : p.ink, { size: 10, align: 'center', weight: 700 });
    },
    [s, node],
  );

  const rootConflicts = s.cbs.nodes[0] ? countConflicts(s.cbs.nodes[0].paths) : 0;

  return (
    <WidgetFrame
      id="w14.4"
      title="Constraint Tree"
      teaches="Optimal multi-agent paths do not require searching the joint space: CBS searches a tree of constraints whose size tracks the number of conflicts, not 5 to the number of agents."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Left: the room, the agents replaying the paths of the highlighted constraint-tree node; the red ring is
          that node’s first conflict. Right: the tree. The root plans every agent alone; each expansion finds the
          first conflict and spawns two children, one forbidding the cell (or arc) to each agent at that time,
          re-planning only that agent with Chapter 6’s A* over (cell, time). Nodes are popped in order of SIC,
          so the first conflict-free node popped is optimal. The readout puts the tree’s size against the
          joint-state successors a coupled A* generates on the same instance.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.1} padding={0.03} draw={draw} deps={[sim.tick, s]} ariaLabel="A grid with several disc agents replaying conflict-based-search paths, beside the constraint tree with sum-of-costs labels." />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="CT expansions · nodes" value={`${s.cbs.expansions} · ${s.cbs.nodes.length}`} role="graph" />
          <StatTile label="SIC (root → solution)" value={`${s.cbs.nodes[0]?.cost ?? '—'} → ${s.cbs.solution ? s.cbs.solution.cost : '…'}`} role="path" />
          <StatTile label="conflicts at the root" value={rootConflicts} precision={0} role="goal" />
          {showJoint ? (
            <StatTile label="coupled A*: joint successors generated" value={s.joint ? s.joint.generated : `not run — branching 5^${agents} = ${5 ** agents}`} precision={0} role="obstacle" />
          ) : (
            <StatTile label="low-level A* expansions" value={s.cbs.lowLevelExpansions} precision={0} />
          )}
        </Dashboard>
      </div>
      <ControlPanel columns={2} title="headline — number of agents">
        <Slider label="agents" role="robot" value={agents} min={2} max={5} step={1} onChange={setAgents} format={(x) => x.toFixed(0)} />
        <Toggle label="show the coupled A* joint-state count (≤ 3 agents)" checked={showJoint} onChange={setShowJoint} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={stepOnce} onReset={sim.reset} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
