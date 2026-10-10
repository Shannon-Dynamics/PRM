'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import { GridGraph, cellAt, cellCenter, cellIJ, freeCellNear, gridFromWorld, type Connectivity } from '@/lib/search/grid';
import { universalPlanGrid } from '@/lib/search/policy';

/**
 * w6.3 — the Universal Plan Field.
 *
 * Backward Dijkstra from the goal fills the Apartment lattice with cost-to-go
 * (a heightfield in the graph colour, darker near the goal) and one arrow per
 * cell: the successor to take. Kick Rusty anywhere — click — and the arrows
 * bring it home. Hover a cell and the sticky panel evaluates
 * h(X) = min_Y [c(X, Y) + h(Y)] for it, term by term. A plan can be a field,
 * and a field is what survives disturbance.
 */

const CELL = 0.25;

interface State {
  robot: number;
  trail: number[];
  rng: Rng;
  idle: number;
}

export function UniversalPlanField() {
  const [connectivity, setConnectivity] = useState<Connectivity>(8);
  const [goal, setGoal] = useState<number | null>(null);
  const [moveGoal, setMoveGoal] = useState(false);
  const [showArrows, setShowArrows] = useState(true);
  const [hover, setHover] = useState<number | null>(null);
  const pressed = useRef(false);

  const grid = useMemo(() => gridFromWorld(APARTMENT, CELL, 0.22), []);
  const graph = useMemo(() => new GridGraph(grid, { connectivity, diagonal: Math.SQRT2, cornerCutting: false }), [grid, connectivity]);
  const goalCell = goal ?? freeCellNear(grid, 11.0, 8.0);
  const plan = useMemo(() => universalPlanGrid(graph, goalCell), [graph, goalCell]);

  const init = useCallback(
    (seed: number): State => {
      const rng = new Rng(seed);
      return { robot: freeCellNear(grid, 1.0 + rng.uniform(0, 2), 1.0 + rng.uniform(0, 2)), trail: [], rng, idle: 0 };
    },
    [grid],
  );

  const step = useCallback(
    (s: State): State => {
      if (!Number.isFinite(plan.costToGo[s.robot])) return s;
      if (s.robot === plan.goal) {
        // Arrived: pause, then kick Rusty somewhere random so the field keeps demonstrating itself.
        if (s.idle < 12) return { ...s, idle: s.idle + 1 };
        let c = s.robot;
        for (let tries = 0; tries < 50; tries++) {
          c = freeCellNear(grid, s.rng.uniform(0.3, 11.7), s.rng.uniform(0.3, 8.7));
          if (Number.isFinite(plan.costToGo[c]) && plan.costToGo[c] > 3) break;
        }
        return { ...s, robot: c, trail: [], idle: 0 };
      }
      const next = plan.next[s.robot];
      if (next < 0) return s;
      return { ...s, robot: next, trail: [...s.trail, s.robot].slice(-400), idle: 0 };
    },
    [plan, grid],
  );

  const sim = useSimulation<State>({ init, step, fps: 12, initialSeed: 63 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [connectivity, goalCell]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const c = cellAt(grid, w[0], w[1]);
      if (phase === 'down') {
        pressed.current = true;
        if (moveGoal) {
          if (!grid.occupied[c]) setGoal(c);
        } else if (!grid.occupied[c]) {
          sim.setState((s) => ({ ...s, robot: c, trail: [], idle: 0 }));
        }
      } else if (phase === 'move') {
        setHover(c);
        if (pressed.current && !moveGoal && !grid.occupied[c]) sim.setState((s) => ({ ...s, robot: c, trail: [], idle: 0 }));
      } else pressed.current = false;
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [grid, moveGoal],
  );

  const s = sim.state;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const cw = Math.ceil(sl(v, CELL)) + 1;
      ctx.save();
      // Heightfield: darker (more opaque graph colour) nearer the goal.
      for (let k = 0; k < grid.occupied.length; k++) {
        const c = plan.costToGo[k];
        const { x, y } = cellCenter(grid, k);
        if (grid.occupied[k]) {
          ctx.fillStyle = p.obstacle;
          ctx.globalAlpha = 0.9;
        } else if (!Number.isFinite(c)) {
          ctx.fillStyle = p.unknown;
          ctx.globalAlpha = 0.5;
        } else {
          ctx.fillStyle = p.graph;
          ctx.globalAlpha = 0.08 + 0.6 * (1 - c / plan.maxCost);
        }
        ctx.fillRect(sx(v, x - CELL / 2), sy(v, y + CELL / 2), cw, cw);
      }
      ctx.globalAlpha = 1;
      // Equal-cost contours every metre: the wave fronts of Chapter 7.
      ctx.strokeStyle = p.bg;
      ctx.lineWidth = 1;
      for (let k = 0; k < grid.occupied.length; k++) {
        const c = plan.costToGo[k];
        if (!Number.isFinite(c)) continue;
        const [i, j] = cellIJ(grid, k);
        if (i + 1 < grid.nx) {
          const cr = plan.costToGo[k + 1];
          if (Number.isFinite(cr) && Math.floor(c) !== Math.floor(cr)) {
            const { x, y } = cellCenter(grid, k);
            ctx.beginPath(); ctx.moveTo(sx(v, x + CELL / 2), sy(v, y - CELL / 2)); ctx.lineTo(sx(v, x + CELL / 2), sy(v, y + CELL / 2)); ctx.stroke();
          }
        }
        if (j + 1 < grid.ny) {
          const cu = plan.costToGo[k + grid.nx];
          if (Number.isFinite(cu) && Math.floor(c) !== Math.floor(cu)) {
            const { x, y } = cellCenter(grid, k);
            ctx.beginPath(); ctx.moveTo(sx(v, x - CELL / 2), sy(v, y + CELL / 2)); ctx.lineTo(sx(v, x + CELL / 2), sy(v, y + CELL / 2)); ctx.stroke();
          }
        }
      }
      // Arrows on every other cell.
      if (showArrows) {
        ctx.strokeStyle = p.ink;
        ctx.globalAlpha = 0.55;
        ctx.lineWidth = 1;
        for (let k = 0; k < grid.occupied.length; k++) {
          const [i, j] = cellIJ(grid, k);
          if ((i + j) % 2 !== 0 || plan.next[k] < 0 || k === plan.goal) continue;
          const a = cellCenter(grid, k), b = cellCenter(grid, plan.next[k]);
          const dx = b.x - a.x, dy = b.y - a.y;
          const n = Math.hypot(dx, dy) || 1;
          const ux = (dx / n) * CELL * 0.38, uy = (dy / n) * CELL * 0.38;
          ctx.beginPath();
          ctx.moveTo(sx(v, a.x - ux), sy(v, a.y - uy));
          ctx.lineTo(sx(v, a.x + ux), sy(v, a.y + uy));
          // Head.
          const hx = a.x + ux, hy = a.y + uy;
          const ang = Math.atan2(uy, ux);
          ctx.lineTo(sx(v, hx - 0.08 * Math.cos(ang - 0.5)), sy(v, hy - 0.08 * Math.sin(ang - 0.5)));
          ctx.moveTo(sx(v, hx), sy(v, hy));
          ctx.lineTo(sx(v, hx - 0.08 * Math.cos(ang + 0.5)), sy(v, hy - 0.08 * Math.sin(ang + 0.5)));
          ctx.stroke();
        }
        ctx.globalAlpha = 1;
      }
      drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      // Trail and Rusty.
      if (s.trail.length > 1) {
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 2.5;
        ctx.globalAlpha = 0.8;
        ctx.beginPath();
        s.trail.forEach((c, n) => {
          const { x, y } = cellCenter(grid, c);
          if (n === 0) ctx.moveTo(sx(v, x), sy(v, y)); else ctx.lineTo(sx(v, x), sy(v, y));
        });
        const { x, y } = cellCenter(grid, s.robot);
        ctx.lineTo(sx(v, x), sy(v, y));
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      const rc = cellCenter(grid, s.robot);
      ctx.fillStyle = p.robot;
      ctx.beginPath();
      ctx.arc(sx(v, rc.x), sy(v, rc.y), sl(v, 0.22), 0, Math.PI * 2);
      ctx.fill();
      const gc = cellCenter(grid, plan.goal);
      ctx.fillStyle = p.goal;
      ctx.beginPath();
      ctx.arc(sx(v, gc.x), sy(v, gc.y), sl(v, 0.22), 0, Math.PI * 2);
      ctx.fill();
      // Hover highlight.
      if (hover !== null && !grid.occupied[hover]) {
        const hc = cellCenter(grid, hover);
        ctx.strokeStyle = p.accent;
        ctx.lineWidth = 2;
        ctx.strokeRect(sx(v, hc.x - CELL / 2), sy(v, hc.y + CELL / 2), sl(v, CELL), sl(v, CELL));
        if (Number.isFinite(plan.costToGo[hover])) label(ctx, `h = ${plan.costToGo[hover].toFixed(2)} m`, sx(v, hc.x) + 10, sy(v, hc.y) - 10, p.accent, { size: 10, weight: 700 });
      }
      ctx.restore();
    },
    [grid, plan, s, hover, showArrows],
  );

  // The Bellman equation at the hovered (or robot's) cell.
  const X = hover !== null && !grid.occupied[hover] && Number.isFinite(plan.costToGo[hover]) ? hover : s.robot;
  const terms = useMemo(() => {
    const out = graph.neighbors(X).map(({ to, cost }) => ({ to, cost, h: plan.costToGo[to], sum: cost + plan.costToGo[to] }));
    out.sort((a, b) => a.sum - b.sum);
    return out;
  }, [graph, X, plan]);
  const cellName = (c: number) => { const [i, j] = cellIJ(grid, c); return `(${i},${j})`; };

  return (
    <WidgetFrame
      id="w6.3"
      title="Universal Plan Field"
      teaches="A plan is not a path. A plan can be a field — a successor for every cell — and a field is what survives being pushed off course."
      colorKey={['goal', 'obstacle', 'graph', 'robot']}
      wide
      caption={
        <>
          One backward Dijkstra from the red goal labels every free cell of the Apartment with its
          cost-to-go (darker blue = closer) and a successor arrow. Rusty follows the arrows; when it
          arrives it is kicked somewhere else and comes home again. Click anywhere to kick it
          yourself. The faint contours every metre are equal-cost fronts — Chapter 7&rsquo;s
          wave-front planner is this picture. Hover a cell: the panel evaluates
          h(X) = min<sub>Y</sub> [c(X, Y) + h(Y)] for it, and the minimum is the arrow.
        </>
      }
    >
      <SimCanvas
        world={APARTMENT.bounds}
        draw={draw}
        deps={[sim.tick, s, hover, plan, showArrows]}
        aspect={12 / 9}
        padding={0.3}
        onPointer={onPointer}
        cursor={moveGoal ? 'crosshair' : 'pointer'}
        ariaLabel="The Apartment floorplan shaded by cost-to-goal with small arrows in every cell pointing toward the goal; an orange robot follows the arrows from wherever it is placed."
      />
      <div className="border-t border-fd-border px-3 py-2 font-mono text-[0.78rem] tabular-nums">
        <div className="eyebrow mb-1">the deterministic Bellman equation at X = {cellName(X)}{X === s.robot ? ' (Rusty)' : ' (hover)'}</div>
        <div>
          h(X) = min<sub>Y</sub> [ c(X, Y) + h(Y) ] = min {'{'}
          {terms.map((t, i) => (
            <span key={t.to} className={i === 0 ? 'font-semibold' : 'text-fd-muted-foreground'}>
              {i > 0 ? ', ' : ' '}
              {t.cost.toFixed(2)} + {Number.isFinite(t.h) ? t.h.toFixed(2) : '∞'}
            </span>
          ))}
          {' }'} = <span className="font-semibold">{Number.isFinite(plan.costToGo[X]) ? plan.costToGo[X].toFixed(2) : '∞'}</span>
          {terms.length ? <span className="text-fd-muted-foreground"> — attained by Y = {cellName(terms[0].to)}, the arrow</span> : null}
        </div>
      </div>
      <ControlPanel columns={3}>
        <ButtonRow>
          <ActionButton emphasis={connectivity === 8} onClick={() => setConnectivity(8)}>8-connected</ActionButton>
          <ActionButton emphasis={connectivity === 4} onClick={() => setConnectivity(4)}>4-connected</ActionButton>
        </ButtonRow>
        <Toggle label="Click moves the goal (recomputes the field)" role="goal" checked={moveGoal} onChange={setMoveGoal} />
        <Toggle label="Show arrows" checked={showArrows} onChange={setShowArrows} />
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
