'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Point2 } from '@/lib/sim/world';
import type { Polygon } from '@/lib/collide/polygon';
import { randomConvexField } from '@/lib/bugs/geometry';
import { VisibilityScene, strictlyInside } from '@/lib/roadmap/visibility';
import { FIG_5_8_BOUNDS, FIG_5_8_RECT1, FIG_5_8_RECT2, FIG_5_8_V } from '@/lib/roadmap/examples';
import { marker } from '@/components/ch/ch07/shared';

/**
 * w8.1 — the Rotational Sweep.
 *
 * Choset Fig. 5.8: two rectangles and a lighthouse vertex v. A half-line turns
 * about v; the active list S holds the edges it currently crosses, nearest
 * first, and at every vertex angle the algorithm asks one question — does
 * v·v_i reach v_i before the nearest edge in S? — then inserts or deletes the
 * vertex's two edges. The tables beside the canvas are Table 5.1 filling in
 * live. Click any vertex to make it the lighthouse; add polygons; at the end
 * the whole graph appears with the A* shortest path from start to goal, and
 * the reduced toggle fades every edge that is not on a supporting or
 * separating line.
 */

const GOAL: Point2 = { x: 10.7, y: 0.6 };
const SWEEP_TICKS = 260;

interface State {
  angle: number;
  hold: number;
}

export function RotationalSweep() {
  const [extraPolys, setExtraPolys] = useState<Polygon[]>([]);
  const [lighthouse, setLighthouse] = useState<number | null>(null);
  const [reduced, setReduced] = useState(false);
  const [showGraph, setShowGraph] = useState(true);
  const [scrub, setScrub] = useState<number | null>(null);

  const scene = useMemo(
    () =>
      new VisibilityScene([FIG_5_8_RECT1, FIG_5_8_RECT2, ...extraPolys], [
        { p: FIG_5_8_V, name: 'v' },
        { p: GOAL, name: 'goal' },
      ]),
    [extraPolys],
  );
  const vId = useMemo(() => {
    if (lighthouse !== null && lighthouse < scene.nodes.length && scene.nodes[lighthouse].free) return lighthouse;
    return scene.nodes.findIndex((n) => n.name === 'v');
  }, [scene, lighthouse]);
  const sweep = useMemo(() => scene.rotationalSweep(vId), [scene, vId]);
  const graph = useMemo(() => scene.graph({ reduced: false }), [scene]);
  const reducedGraph = useMemo(() => scene.graph({ reduced: true }), [scene]);
  const path = useMemo(() => {
    const s = scene.nodes.findIndex((n) => n.name === 'v');
    const g = scene.nodes.findIndex((n) => n.name === 'goal');
    return (reduced ? reducedGraph : graph).shortestPath(s, g);
  }, [scene, graph, reducedGraph, reduced]);

  const init = useCallback((): State => ({ angle: 0, hold: 0 }), []);
  const step = useCallback((s: State): State => {
    if (s.angle >= 2 * Math.PI) return s.hold < 50 ? { ...s, hold: s.hold + 1 } : { angle: 0, hold: 0 };
    return { angle: Math.min(2 * Math.PI, s.angle + (2 * Math.PI) / SWEEP_TICKS), hold: 0 };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 58 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [scene, vId]);

  const angle = scrub ?? sim.state.angle;
  const processed = useMemo(() => sweep.events.filter((e) => e.angle <= angle), [sweep, angle]);
  const S = processed.length ? processed[processed.length - 1].S : sweep.initialS;
  const done = angle >= 2 * Math.PI - 1e-9;

  const addPolygon = () => {
    const rng = new Rng(sim.seed + 17 * extraPolys.length);
    const keep = [...scene.nodes.map((n) => n.p)];
    const polys = randomConvexField(rng, { minX: -0.5, minY: -2.6, maxX: 10.5, maxY: 4.6 }, { count: 1, radius: [0.5, 0.9], margin: 0.35, keepFree: keep });
    const pts = polys[0]?.pts;
    if (!pts) return;
    const poly = [...pts];
    // Reject overlaps with the existing polygons (vertices inside, or containing an existing vertex).
    const all = [FIG_5_8_RECT1, FIG_5_8_RECT2, ...extraPolys];
    if (poly.some((p) => all.some((q) => strictlyInside(p, q))) || all.some((q) => q.some((p) => strictlyInside(p, poly)))) return;
    setExtraPolys([...extraPolys, poly]);
  };

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase !== 'down') return;
      let best = -1;
      let bd = 0.35;
      for (const n of scene.nodes) {
        if (!n.free || n.name === 'goal') continue;
        const d = Math.hypot(n.p.x - w[0], n.p.y - w[1]);
        if (d < bd) {
          bd = d;
          best = n.id;
        }
      }
      if (best >= 0) {
        setLighthouse(best);
        setScrub(null);
      }
    },
    [scene],
  );

  const vp = scene.nodes[vId].p;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      // Obstacles.
      ctx.save();
      ctx.fillStyle = p.cobstacle;
      ctx.globalAlpha = 0.28;
      for (const poly of scene.polys) {
        ctx.beginPath();
        poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
        ctx.closePath();
        ctx.fill();
      }
      ctx.globalAlpha = 1;
      ctx.strokeStyle = p.cobstacle;
      ctx.lineWidth = 1.5;
      for (const poly of scene.polys) {
        ctx.beginPath();
        poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
        ctx.closePath();
        ctx.stroke();
      }
      ctx.restore();
      // The whole graph at the end (light), reduced edges solid.
      if (done && showGraph) {
        ctx.save();
        ctx.lineWidth = 1;
        for (const e of graph.edges()) {
          const kept = reducedGraph.hasEdge(e.a, e.b);
          ctx.strokeStyle = p.graph;
          ctx.globalAlpha = reduced ? (kept ? 0.55 : 0.08) : 0.3;
          ctx.beginPath();
          ctx.moveTo(sx(v, e.points[0].x), sy(v, e.points[0].y));
          ctx.lineTo(sx(v, e.points[1].x), sy(v, e.points[1].y));
          ctx.stroke();
        }
        ctx.restore();
        if (path) {
          ctx.save();
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 3;
          ctx.beginPath();
          path.points.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
          ctx.stroke();
          ctx.restore();
        }
      }
      // Edges in S: the crossed edges, nearest first.
      ctx.save();
      S.forEach((id, k) => {
        const e = scene.edges[id];
        ctx.strokeStyle = k === 0 ? p.goal : p.graph;
        ctx.lineWidth = k === 0 ? 4 : 3;
        ctx.beginPath();
        ctx.moveTo(sx(v, e.seg.x1), sy(v, e.seg.y1));
        ctx.lineTo(sx(v, e.seg.x2), sy(v, e.seg.y2));
        ctx.stroke();
      });
      ctx.restore();
      // Visible edges found so far.
      ctx.save();
      ctx.strokeStyle = p.graph;
      ctx.lineWidth = 1.8;
      for (const e of processed) {
        if (!e.visible) continue;
        const q = scene.nodes[e.node].p;
        ctx.beginPath();
        ctx.moveTo(sx(v, vp.x), sy(v, vp.y));
        ctx.lineTo(sx(v, q.x), sy(v, q.y));
        ctx.stroke();
      }
      ctx.restore();
      // The sweep line l.
      const L = 30;
      ctx.save();
      ctx.strokeStyle = p.accent;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      ctx.moveTo(sx(v, vp.x), sy(v, vp.y));
      ctx.lineTo(sx(v, vp.x + L * Math.cos(angle)), sy(v, vp.y + L * Math.sin(angle)));
      ctx.stroke();
      ctx.restore();
      // Nodes and names.
      for (const n of scene.nodes) {
        if (!n.free) continue;
        const px = sx(v, n.p.x);
        const py = sy(v, n.p.y);
        const ev = processed.find((e) => e.node === n.id);
        const isV = n.id === vId;
        const col = isV ? p.start : n.name === 'goal' ? p.goal : ev ? (ev.visible ? p.graph : p.obstacle) : p.ink;
        marker(ctx, px, py, isV ? 7 : 4.5, col, p.bg);
        label(ctx, n.name, px + 7, py - 8, col, { size: 10, weight: isV ? 700 : 500 });
      }
      label(ctx, done ? `sweep complete: ${sweep.visible.length} vertices visible from ${scene.nodes[vId].name}${path ? ` · shortest path v → goal ${path.length.toFixed(3)} on the ${reduced ? 'reduced' : 'full'} graph` : ''}` : `l at ${((angle * 180) / Math.PI).toFixed(1)}° · S = {${S.map((id) => scene.edges[id].name).join(', ')}}`, 10, 14, p.ink, { size: 10.5, weight: 600 });
      label(ctx, 'click a vertex to move the lighthouse', 10, v.height - 12, p.ink, { size: 9 });
    },
    [scene, S, processed, angle, vp, vId, done, showGraph, graph, reducedGraph, reduced, path, sweep],
  );

  const rows = sweep.events;

  return (
    <WidgetFrame
      id="w8.1"
      title="Rotational Sweep"
      teaches="Visibility does not need O(n) tests per pair: a turning half-line keeps the crossed edges sorted, and only the nearest one can occlude. The reduced graph drops needless edges and loses no shortest path."
      colorKey={['start', 'goal', 'cobstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Choset Fig. 5.8 with the lighthouse at v (green). The dashed half-line turns; the edges it
          crosses are the active list S (blue, the nearest in red), and at each vertex angle the
          vertex is visible iff the segment to it is shorter than the distance to that nearest edge
          and the line does not enter the obstacle at v. The table is Table 5.1 filling in: the
          initial S is {'{E4, E2, E8, E6}'}, and the three edges added are (v, v4), (v, v8), (v, v1).
          When the sweep completes the full visibility graph appears with the A* path from v to the
          goal; the <em>reduced</em> toggle fades every edge not on a supporting or separating line
          and the path length does not change. Click any vertex to make it the lighthouse, add
          polygons, or scrub the angle.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border lg:grid-cols-[3fr_2fr]">
        <div className="bg-fd-card">
          <SimCanvas world={FIG_5_8_BOUNDS} draw={draw} deps={[sim.tick, angle, scene, vId, reduced, showGraph, scrub]} aspect={12 / 8} padding={0.3} onPointer={onPointer} cursor="pointer" ariaLabel="Two amber rectangles and a green lighthouse vertex; a dashed half-line sweeps around it while the crossed edges highlight in blue and visible vertices connect to the lighthouse." />
        </div>
        <div className="max-h-[420px] overflow-auto bg-fd-card px-3 py-2 font-mono text-[0.68rem] tabular-nums">
          <div className="eyebrow mb-1">Table 5.1 — vertex · new S · actions</div>
          <table className="w-full">
            <tbody>
              <tr className="text-fd-muted-foreground">
                <td className="py-0.5 pr-2">init</td>
                <td className="py-0.5 pr-2">{`{${sweep.initialS.map((id) => scene.edges[id].name).join(', ')}}`}</td>
                <td className="py-0.5">sort edges crossing the horizontal half-line</td>
              </tr>
              {rows.map((e, k) => {
                const live = e.angle <= angle;
                const n = scene.nodes[e.node];
                return (
                  <tr key={k} className={live ? '' : 'opacity-35'} style={live && e.visible ? { color: 'var(--rm-graph)' } : undefined}>
                    <td className="py-0.5 pr-2 align-top">α{n.name.replace(/^v/, '')} {((e.angle * 180) / Math.PI).toFixed(1)}°</td>
                    <td className="py-0.5 pr-2 align-top">{`{${e.S.map((id) => scene.edges[id].name).join(', ')}}`}</td>
                    <td className="py-0.5 align-top">
                      {e.deleted.map((id) => `Delete ${scene.edges[id].name}`).join('. ')}
                      {e.deleted.length && e.inserted.length ? '. ' : ''}
                      {e.inserted.map((id) => `Add ${scene.edges[id].name}`).join('. ')}
                      {e.visible ? <span className="font-semibold"> ADD ({scene.nodes[vId].name}, {n.name})</span> : null}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      </div>
      <ControlPanel columns={3} title="Headline: the lighthouse vertex (click) · the sweep angle">
        <Slider label="sweep angle (scrub; release to resume)" value={(angle * 180) / Math.PI} min={0} max={360} step={0.5} unit="°" onChange={(v) => { sim.pause(); setScrub((v * Math.PI) / 180); }} />
        <ButtonRow>
          <ActionButton onClick={addPolygon}>add a polygon</ActionButton>
          <ActionButton onClick={() => { setExtraPolys([]); setLighthouse(null); setScrub(null); }}>Fig. 5.8 only</ActionButton>
        </ButtonRow>
        <div className="flex flex-col gap-1.5">
          <Toggle label="Reduced graph (supporting and separating lines only)" role="graph" checked={reduced} onChange={setReduced} />
          <Toggle label="Show the full graph when the sweep completes" role="graph" checked={showGraph} onChange={setShowGraph} />
        </div>
      </ControlPanel>
      <div className="border-t border-fd-border px-3 py-2 font-mono text-[0.7rem] text-fd-muted-foreground tabular-nums">
        full graph {graph.edgeCount} edges · reduced {reducedGraph.edgeCount} edges · {scene.nodes.filter((n) => n.free).length} nodes
        {path ? ` · v → goal: ${path.length.toFixed(3)} m, ${path.expansions} A* expansions` : ' · no path'}
      </div>
      <Transport playing={sim.playing} onToggle={() => { setScrub(null); sim.toggle(); }} onStep={sim.stepOnce} onReset={() => { setScrub(null); sim.reset(); }} onReseed={() => { setScrub(null); sim.reseed(); }} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
