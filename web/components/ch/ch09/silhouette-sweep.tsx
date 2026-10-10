'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Vec } from '@/lib/roadmap/gvg';
import { cannySweep, drilledEllipsoid, permuteAxes, sphere, torus, type ImplicitSurface, type Silhouette } from '@/lib/roadmap/silhouette';
import { diamond, marker } from '@/components/ch/ch07/shared';
import { point3, polyline3, project, quad3, type Camera } from './shared';

/**
 * w9.2 — the Silhouette Sweep (Choset §5.5.1).
 *
 * A slicing plane q₁ = λ sweeps a surface in R³ (a torus, C Figs. 5.31–5.32;
 * an ellipsoid with a bent hole, C Fig. 5.25; a sphere). On each slice the
 * surface is a set of closed curves, and the extrema of q₂ on them are the
 * silhouette points (Lemma 5.5.2: D(f, π₁₂) loses rank). As λ advances the
 * extrema trace the silhouette curves; where the number of slice curves or of
 * extrema changes — a critical slice — the curves break, and Canny's
 * algorithm recurses on that slice. Its one-dimensional roadmap is the slice
 * curve itself: the linking curves. Everything drawn comes from `cannySweep`
 * in `lib/roadmap/silhouette.ts`, computed once per body and direction.
 */

type BodyName = 'torus' | 'ellipsoid' | 'sphere';
type SweepAxis = 0 | 1;

const BODIES: Record<BodyName, { make: () => ImplicitSurface; n: number; seeds: number; label: string }> = {
  torus: { make: () => torus(2, 0.7), n: 90, seeds: 12, label: 'torus (R = 2, r = 0.7)' },
  ellipsoid: { make: () => drilledEllipsoid(), n: 110, seeds: 14, label: 'ellipsoid with a bent hole' },
  sphere: { make: () => sphere(1.6), n: 60, seeds: 10, label: 'sphere' },
};

/** Sweep along q₁ (identity) or along q₂ (swap the first two axes). */
const PERMS: Record<SweepAxis, [number, number, number]> = { 0: [0, 1, 2], 1: [1, 0, 2] };

/** Swept coordinates q' back to the body's own frame, so the camera never moves when the sweep does. */
function toOrig(qp: Vec, perm: [number, number, number]): Vec {
  const q = [0, 0, 0];
  for (let i = 0; i < 3; i++) q[perm[i]] = qp[i];
  return q;
}

const HOLD = 30; // ticks to linger at the end of a sweep

interface State {
  k: number;
}

export function SilhouetteSweep() {
  const [body, setBody] = useState<BodyName>('torus');
  const [axis, setAxis] = useState<SweepAxis>(0);
  const [recursion, setRecursion] = useState(true);
  const [wire, setWire] = useState(true);
  const [yaw, setYaw] = useState(-0.62);

  // One sweep per (body, direction); a few hundred milliseconds, then every frame is drawing only.
  const cache = useRef(new Map<string, Silhouette>());
  const key = `${body}:${axis}`;
  const sil = useMemo(() => {
    const hit = cache.current.get(key);
    if (hit) return hit;
    const b = BODIES[body];
    const surf = axis === 0 ? b.make() : permuteAxes(b.make(), PERMS[axis]);
    const s = cannySweep(surf, b.n, { step: 0.03, seeds: b.seeds });
    cache.current.set(key, s);
    return s;
  }, [key, body, axis]);

  const perm = PERMS[axis];
  const n = sil.lambdas.length;
  const nRef = useRef(n);
  nRef.current = n;

  const init = useCallback((): State => ({ k: 0 }), []);
  const step = useCallback((s: State): State => ({ k: s.k + 1 >= nRef.current + HOLD ? 0 : s.k + 1 }), []);
  const sim = useSimulation<State>({ init, step, fps: 14, initialSeed: 0 });

  const k = Math.min(sim.state.k, n - 1);
  const lam = sil.lambdas[k];
  const slice = sil.slices[k];
  const bounds = sil.surface.bounds;

  // The critical slice the plane is at (within one step) or most recently passed.
  const dl = sil.lambdas[1] - sil.lambdas[0];
  const passed = sil.critical.filter((c) => c.lambda <= lam + 1e-9);
  const atCritical = sil.critical.find((c) => Math.abs(c.lambda - lam) <= dl);

  const setK = useCallback(
    (v: number) => {
      sim.pause();
      sim.setState(() => ({ k: Math.max(0, Math.min(nRef.current - 1, Math.round(v))) }));
    },
    [sim],
  );

  const jumpToNextCritical = useCallback(() => {
    const next = sil.critical.find((c) => c.lambda > lam + 0.5 * dl) ?? sil.critical[0];
    if (!next) return;
    let best = 0;
    sil.lambdas.forEach((l, i) => {
      if (Math.abs(l - next.lambda) < Math.abs(sil.lambdas[best] - next.lambda)) best = i;
    });
    setK(best);
  }, [sil, lam, dl, setK]);

  const draw3d = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const ext = Math.max(...bounds.map(([a, b]) => Math.max(Math.abs(a), Math.abs(b))));
      const cam: Camera = { yaw, pitch: 0.5, scale: (0.4 * Math.min(v.width, v.height * 1.25)) / ext, cx: v.width / 2, cy: v.height / 2 + 6 };
      const O = (q: Vec) => toOrig(q, perm);

      // Wireframe: every few slices' curves, faint. The body is drawn by the very curves the sweep traced.
      if (wire) {
        const stride = Math.max(1, Math.round(n / 22));
        for (let i = 0; i < n; i += stride) {
          for (const pc of sil.slices[i].pieces) polyline3(ctx, cam, pc.points.map((pt) => O([sil.lambdas[i], pt.x, pt.y])), p.obstacle, 1, 0.22);
        }
      }

      // The sweep plane at λ.
      const [b2, b3] = [bounds[1], bounds[2]];
      const pad = 0.25;
      const corners = [
        [lam, b2[0] - pad, b3[0] - pad],
        [lam, b2[1] + pad, b3[0] - pad],
        [lam, b2[1] + pad, b3[1] + pad],
        [lam, b2[0] - pad, b3[1] + pad],
      ].map(O);
      quad3(ctx, cam, corners, p.accent, p.accent, 0.08);

      // Silhouette curves, revealed up to the plane.
      for (const c of sil.curves) {
        const shown = c.points.filter((q) => q[0] <= lam + 1e-9).map(O);
        polyline3(ctx, cam, shown, p.graph, 2.4, 0.95);
      }

      // Linking curves: the recursion's roadmap on each critical slice the plane has passed.
      if (recursion) {
        for (const c of passed) for (const L of c.linking) polyline3(ctx, cam, L.map(O), p.graph, 1.6, 0.85, true);
      }

      // The current slice and its extrema.
      for (const pc of slice.pieces) polyline3(ctx, cam, pc.points.map((pt) => O([lam, pt.x, pt.y])), p.accent, 2.2, 0.95);
      for (const e of slice.extrema) point3(ctx, cam, O(e.q), 4, e.max ? p.graph : p.bg, p.graph);

      // Critical points passed so far.
      for (const c of passed) {
        const s = project(cam, O(c.point));
        diamond(ctx, s.x, s.y, 6, p.ink);
      }

      // Axes triad, bottom left.
      const ox = 34;
      const oy = v.height - 26;
      const axes: [Vec, string][] = [
        [[1, 0, 0], 'q₁'],
        [[0, 1, 0], 'q₂'],
        [[0, 0, 1], 'q₃'],
      ];
      for (const [d, name] of axes) {
        const a = project({ ...cam, cx: ox, cy: oy, scale: 18 }, [0, 0, 0]);
        const b = project({ ...cam, cx: ox, cy: oy, scale: 18 }, d);
        ctx.save();
        ctx.strokeStyle = p.ink;
        ctx.globalAlpha = 0.6;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.restore();
        label(ctx, name, b.x + (b.x - a.x) * 0.35, b.y + (b.y - a.y) * 0.35, p.ink, { size: 9, align: 'center' });
      }

      const sweepName = axis === 0 ? 'q₁' : 'q₂';
      label(ctx, `${BODIES[body].label} · plane ${sweepName} = λ = ${lam.toFixed(2)}`, 10, 14, p.ink, { size: 10.5, weight: 600 });
      label(
        ctx,
        `silhouette components: ${sil.componentsWithoutLinks} alone → ${sil.componentsWithLinks} with linking curves · ${sil.critical.length} critical slices`,
        10,
        28,
        p.ink,
        { size: 9.5 },
      );
      if (atCritical) label(ctx, `critical slice λ* = ${atCritical.lambda.toFixed(3)}: (curves, extrema) ${atCritical.before.join(',')} → ${atCritical.after.join(',')}`, 10, 42, p.ink, { size: 9.5, weight: 600 });
    },
    [sil, perm, n, lam, slice, passed, atCritical, recursion, wire, yaw, bounds, body, axis],
  );

  const drawSlice = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const W = v.width;
      const H = v.height;
      // Upper 68 %: the slice itself, in (q₂', q₃') — the plane seen face-on.
      const [b2, b3] = [bounds[1], bounds[2]];
      const top = 22;
      const sliceH = H * 0.66 - top;
      const span2 = b2[1] - b2[0] + 0.6;
      const span3 = b3[1] - b3[0] + 0.6;
      const s = Math.min((W - 20) / span2, sliceH / span3);
      const cx = W / 2;
      const cy = top + sliceH / 2;
      const m2 = (b2[0] + b2[1]) / 2;
      const m3 = (b3[0] + b3[1]) / 2;
      const px = (x: number) => cx + (x - m2) * s;
      const py = (y: number) => cy - (y - m3) * s;
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.strokeRect(px(b2[0] - 0.3), py(b3[1] + 0.3), span2 * s, span3 * s);
      ctx.restore();
      for (const pc of slice.pieces) {
        ctx.save();
        ctx.strokeStyle = p.accent;
        ctx.lineWidth = 2;
        ctx.beginPath();
        pc.points.forEach((pt, i) => (i === 0 ? ctx.moveTo(px(pt.x), py(pt.y)) : ctx.lineTo(px(pt.x), py(pt.y))));
        ctx.stroke();
        ctx.restore();
      }
      for (const e of slice.extrema) {
        // The silhouette condition on the slice: the curve's tangent is vertical — no q₂ component.
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.globalAlpha = 0.5;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(px(e.q[1]), py(e.q[2]) - 12);
        ctx.lineTo(px(e.q[1]), py(e.q[2]) + 12);
        ctx.stroke();
        ctx.restore();
        marker(ctx, px(e.q[1]), py(e.q[2]), 4, e.max ? p.graph : p.bg, p.graph);
      }
      const h = axis === 0 ? 'q₂' : 'q₁';
      label(ctx, `slice ${axis === 0 ? 'q₁' : 'q₂'} = ${lam.toFixed(2)}: ${slice.pieces.length} curve${slice.pieces.length === 1 ? '' : 's'}, ${slice.extrema.length} extrema of ${h}`, 8, 11, p.ink, { size: 9.5, weight: 600 });
      label(ctx, `${h} →`, W - 8, cy + (span3 * s) / 2 + 10, p.ink, { size: 9, align: 'right' });

      // Lower strip: curve count and extremum count against λ — they change only at critical slices.
      const y0 = H * 0.72;
      const y1 = H - 14;
      const x0 = 26;
      const x1 = W - 8;
      const lamMin = sil.lambdas[0];
      const lamMax = sil.lambdas[n - 1];
      const X = (l: number) => x0 + ((l - lamMin) / (lamMax - lamMin)) * (x1 - x0);
      const maxE = Math.max(2, ...sil.slices.map((sl) => sl.extrema.length));
      const Y = (c: number) => y1 - (c / maxE) * (y1 - y0);
      const series: [(i: number) => number, string, number][] = [
        [(i) => sil.slices[i].extrema.length, p.graph, 1.8],
        [(i) => sil.slices[i].pieces.length, p.accent, 2.2],
      ];
      for (const [f, color, w] of series) {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = w;
        ctx.beginPath();
        for (let i = 0; i < n; i++) {
          const xx = X(sil.lambdas[i]);
          const yy = Y(f(i));
          if (i === 0) ctx.moveTo(xx, yy);
          else {
            ctx.lineTo(xx, Y(f(i - 1)));
            ctx.lineTo(xx, yy);
          }
        }
        ctx.stroke();
        ctx.restore();
      }
      for (const c of sil.critical) {
        ctx.save();
        ctx.strokeStyle = p.ink;
        ctx.globalAlpha = 0.35;
        ctx.setLineDash([2, 3]);
        ctx.beginPath();
        ctx.moveTo(X(c.lambda), y0 - 2);
        ctx.lineTo(X(c.lambda), y1);
        ctx.stroke();
        ctx.restore();
      }
      ctx.save();
      ctx.strokeStyle = p.ink;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(X(lam), y0 - 6);
      ctx.lineTo(X(lam), y1 + 2);
      ctx.stroke();
      ctx.restore();
      label(ctx, 'curves', 4, Y(slice.pieces.length), p.accent, { size: 8.5 });
      label(ctx, `λ`, x1, y1 + 8, p.ink, { size: 9, align: 'right' });
      label(ctx, 'extrema (blue) · curves (teal) · dashed: critical λ', x0, y0 - 10, p.ink, { size: 8.5 });
    },
    [sil, slice, lam, n, bounds, axis],
  );

  const ariaBody = BODIES[body].label;

  return (
    <WidgetFrame
      id="w9.2"
      title="Silhouette Sweep"
      teaches="The silhouette is not the outline: it is the critical set of a projection, and the roadmap's connectivity changes only at critical slices — where the algorithm recurses."
      colorKey={['obstacle', 'graph']}
      wide
      caption={
        <>
          A plane (teal) sweeps the body; its intersection with the surface is the teal slice curve, and
          the blue dots are the extrema of the next coordinate on it — filled for maxima, hollow for
          minima. Those are the points where the slice curve&rsquo;s tangent has no component along that
          coordinate, which is Lemma 5.5.2&rsquo;s rank loss of D(f, π₁₂). As the plane advances the dots
          trace the silhouette curves (solid blue). Watch the strip under the slice view: the number of
          curves and of extrema is constant between the dashed critical values and jumps only at them.
          At each critical slice (black diamond) the silhouette breaks, and Canny&rsquo;s algorithm
          recurses on that slice — whose roadmap is the slice curve itself, drawn dashed: the linking
          curves. Turn the recursion off and the readout says how many pieces the silhouette alone leaves
          disconnected. On the torus, the slice is one curve beyond |λ| = R − r = 1.3 and two inside it
          (C Fig. 5.31).
        </>
      }
    >
      <div className="grid grid-cols-1 sm:grid-cols-[3fr_2fr]">
        <SimCanvas world={{ minX: 0, minY: 0, maxX: 1, maxY: 1 }} draw={draw3d} deps={[sim.tick, k, draw3d]} aspect={4 / 3} padding={0} ariaLabel={`A ${ariaBody} drawn as a wireframe of slice curves, with a translucent plane sweeping through it and blue silhouette curves traced by the extrema on each slice.`} />
        <SimCanvas world={{ minX: 0, minY: 0, maxX: 1, maxY: 1 }} draw={drawSlice} deps={[sim.tick, k, drawSlice]} aspect={4 / 4.2} padding={0} ariaLabel="The current slice seen face-on, with its extrema marked, above a step plot of the number of slice curves and extrema against the sweep parameter." />
      </div>
      <ControlPanel columns={2} title="Headline: the sweep parameter λ">
        <Slider label={`λ (plane ${axis === 0 ? 'q₁' : 'q₂'} = λ)`} role="graph" value={k} min={0} max={n - 1} step={1} format={() => lam.toFixed(2)} onChange={setK} />
        <ButtonRow>
          <ActionButton onClick={() => setBody('torus')} emphasis={body === 'torus'}>torus</ActionButton>
          <ActionButton onClick={() => setBody('ellipsoid')} emphasis={body === 'ellipsoid'}>bent-hole ellipsoid</ActionButton>
          <ActionButton onClick={() => setBody('sphere')} emphasis={body === 'sphere'}>sphere</ActionButton>
          <ActionButton onClick={jumpToNextCritical}>next critical slice</ActionButton>
        </ButtonRow>
      </ControlPanel>
      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Sweep direction · recursion · view</summary>
        <ControlPanel columns={3}>
          <ButtonRow>
            <ActionButton onClick={() => setAxis(0)} emphasis={axis === 0}>sweep along q₁</ActionButton>
            <ActionButton onClick={() => setAxis(1)} emphasis={axis === 1}>sweep along q₂</ActionButton>
          </ButtonRow>
          <div className="flex flex-col gap-1.5">
            <Toggle label="recursion: linking curves on critical slices" role="graph" checked={recursion} onChange={setRecursion} />
            <Toggle label="wireframe" role="obstacle" checked={wire} onChange={setWire} />
          </div>
          <Slider label="view yaw" value={yaw} min={-1.5} max={1.5} step={0.02} unit="rad" onChange={setYaw} />
        </ControlPanel>
      </details>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={k} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
