'use client';

import { useCallback, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import {
  FOUR_CHART_ATLAS,
  SINGLE_CHART_ATTEMPT,
  TWO_CHART_ATLAS,
  chartsContaining,
  embedS1,
  transitionMap,
  wrapAngle,
  type S1ChartDef,
} from '@/lib/manifold/s1';

/**
 * w5.3 — the Atlas of the Circle.
 *
 * Choset's four charts on S¹ (figure 3.14) as coloured arcs, each with its own
 * number line below. A point moves around the circle and is handed from chart
 * to chart; on an overlap both coordinates are shown and the transition map
 * φᵢ ∘ φⱼ⁻¹ = ±√(1 − z²) is plotted with the current point marked. The two-chart
 * angle atlas is the one `S1` uses; the single-chart "attempt" θ ∈ [0, 2π) has
 * a seam where the coordinate tears. An angle is a point on S¹; a number is
 * its coordinate in a chart.
 */

type Mode = 'four' | 'two' | 'single';

interface SimState {
  theta: number;
}

const ATLASES: Record<Mode, readonly S1ChartDef[]> = {
  four: FOUR_CHART_ATLAS,
  two: TWO_CHART_ATLAS,
  single: [SINGLE_CHART_ATTEMPT],
};

export function AtlasOfTheCircle() {
  const [mode, setMode] = useState<Mode>('four');
  const [manual, setManual] = useState<number | null>(null);
  const dragging = useRef(false);

  const init = useCallback((): SimState => ({ theta: 0.6 }), []);
  const step = useCallback((s: SimState): SimState => ({ theta: wrapAngle(s.theta + 0.012) }), []);
  const sim = useSimulation<SimState>({ init, step, fps: 30, initialSeed: 53 });
  const theta = manual ?? sim.state.theta;
  const atlas = ATLASES[mode];

  // Circle centre and radius in canvas-world units (world is 2 × 1).
  const CX = 0.5, CY = 0.52, R = 0.34;

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'down') {
        if (Math.hypot(w[0] - CX, w[1] - CY) < R + 0.2) {
          dragging.current = true;
          sim.pause();
          setManual(Math.atan2(w[1] - CY, w[0] - CX));
        }
      } else if (phase === 'move') {
        if (dragging.current) setManual(Math.atan2(w[1] - CY, w[0] - CX));
      } else dragging.current = false;
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const X = (x: number) => sx(v, x), Y = (y: number) => sy(v, y);
      const px = (r: number) => sx(v, r) - sx(v, 0);
      const containing = chartsContaining(atlas, theta);

      /* ---- the circle and the chart domains as arcs ---- */
      ctx.save();
      ctx.strokeStyle = p.ink;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(X(CX), Y(CY), px(R), 0, Math.PI * 2);
      ctx.stroke();
      // Axes.
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(X(CX - R - 0.08), Y(CY)); ctx.lineTo(X(CX + R + 0.08), Y(CY));
      ctx.moveTo(X(CX), Y(CY - R - 0.08)); ctx.lineTo(X(CX), Y(CY + R + 0.08));
      ctx.stroke();
      label(ctx, 'x₁', X(CX + R + 0.1), Y(CY), p.ink, { size: 10 });
      label(ctx, 'x₂', X(CX), Y(CY + R + 0.11), p.ink, { size: 10, align: 'center' });

      // Each chart domain: an arc at its own radius, drawn by sampling `contains`.
      atlas.forEach((chart, ci) => {
        const rr = R + 0.035 + ci * 0.028;
        const alpha = 0.95 - ci * 0.18;
        const active = chart.contains(theta);
        ctx.strokeStyle = p.accent;
        ctx.globalAlpha = active ? alpha : alpha * 0.45;
        ctx.lineWidth = active ? 6 : 4;
        ctx.beginPath();
        let pen = false;
        for (let k = 0; k <= 360; k++) {
          const th = -Math.PI + (k / 360) * 2 * Math.PI;
          const inside = chart.contains(th);
          const [ex, ey] = embedS1(th);
          if (inside) {
            if (!pen) { ctx.moveTo(X(CX + rr * ex), Y(CY + rr * ey)); pen = true; } else ctx.lineTo(X(CX + rr * ex), Y(CY + rr * ey));
          } else pen = false;
        }
        ctx.stroke();
        ctx.globalAlpha = 1;
      });
      // The seam of the single-chart attempt.
      if (mode === 'single') {
        ctx.strokeStyle = p.goal;
        ctx.setLineDash([4, 3]);
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(X(CX + R - 0.02), Y(CY)); ctx.lineTo(X(CX + R + 0.08), Y(CY));
        ctx.stroke();
        ctx.setLineDash([]);
        label(ctx, 'seam: θ = 0 ≡ 2π', X(CX + R + 0.1), Y(CY + 0.03), p.goal, { size: 9 });
      }
      // The point.
      const [ex, ey] = embedS1(theta);
      ctx.fillStyle = p.robot;
      ctx.beginPath();
      ctx.arc(X(CX + R * ex), Y(CY + R * ey), 7, 0, Math.PI * 2);
      ctx.fill();
      label(ctx, `x = (${ex.toFixed(2)}, ${ey.toFixed(2)})`, X(CX), Y(0.06), p.ink, { size: 11, align: 'center', weight: 600 });
      label(ctx, `in ${containing.length} chart${containing.length === 1 ? '' : 's'}: ${containing.map((c) => c.id).join(', ') || '—'}`, X(CX), Y(0.02), containing.length ? p.ink : p.goal, { size: 10, align: 'center' });
      ctx.restore();

      /* ---- number lines, right half ---- */
      const x0 = X(1.1), x1 = X(1.9);
      const n = atlas.length;
      const gap = (Y(0.3) - Y(0.96)) / Math.max(n, 2);
      atlas.forEach((chart, ci) => {
        const yl = Y(0.96) + gap * (ci + 0.6);
        const [lo, hi] = chart.range;
        const Xz = (z: number) => x0 + ((z - lo) / (hi - lo)) * (x1 - x0);
        ctx.save();
        const active = chart.contains(theta);
        ctx.strokeStyle = active ? p.accent : p.grid;
        ctx.lineWidth = active ? 3 : 1.5;
        ctx.beginPath();
        ctx.moveTo(x0, yl); ctx.lineTo(x1, yl);
        ctx.stroke();
        ctx.strokeStyle = p.ink;
        ctx.lineWidth = 1;
        for (const z of [lo, hi]) { ctx.beginPath(); ctx.moveTo(Xz(z), yl - 5); ctx.lineTo(Xz(z), yl + 5); ctx.stroke(); }
        label(ctx, lo === -1 ? '−1' : lo === 0 ? '0' : '−π', x0, yl + 13, p.ink, { size: 9, align: 'center' });
        label(ctx, hi === 1 ? '1' : hi > 6 ? '2π' : 'π', x1, yl + 13, p.ink, { size: 9, align: 'center' });
        label(ctx, chart.label, x0, yl - 14, active ? p.ink : p.truth, { size: 9.5, weight: active ? 700 : 500 });
        const z = chart.coord(theta);
        if (z !== null) {
          ctx.fillStyle = p.robot;
          ctx.beginPath();
          ctx.arc(Xz(z), yl, 5.5, 0, Math.PI * 2);
          ctx.fill();
          label(ctx, `${chart.id.replace('U', 'φ').replace('V', 'ψ').replace('W', 'θ')} = ${z.toFixed(3)}`, Xz(z) + 9, yl - 1, p.robot, { size: 10, weight: 700 });
        } else {
          label(ctx, 'outside domain', x1 - 2, yl - 1, p.truth, { size: 9, align: 'right' });
        }
        ctx.restore();
      });

      /* ---- transition map on the current overlap ---- */
      const box = { x0: X(1.12), y0: Y(0.26), x1: X(1.42), y1: Y(0.03) };
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.strokeRect(box.x0, box.y0, box.x1 - box.x0, box.y1 - box.y0);
      if (containing.length >= 2) {
        const ci = atlas.indexOf(containing[0]);
        const cj = atlas.indexOf(containing[1]);
        const cjDef = atlas[cj];
        const [lo, hi] = cjDef.range;
        const [loI, hiI] = atlas[ci].range;
        const Xz = (z: number) => box.x0 + ((z - lo) / (hi - lo)) * (box.x1 - box.x0);
        const Yz = (w: number) => box.y0 + ((w - loI) / (hiI - loI)) * (box.y1 - box.y0);
        ctx.strokeStyle = p.accent;
        ctx.lineWidth = 2;
        ctx.beginPath();
        let pen = false;
        for (let k = 1; k < 200; k++) {
          const z = lo + (k / 200) * (hi - lo);
          const w = transitionMap(atlas, ci, cj, z);
          if (w === null) { pen = false; continue; }
          if (!pen) { ctx.moveTo(Xz(z), Yz(w)); pen = true; } else ctx.lineTo(Xz(z), Yz(w));
        }
        ctx.stroke();
        const zc = cjDef.coord(theta)!;
        const wc = atlas[ci].coord(theta)!;
        ctx.fillStyle = p.robot;
        ctx.beginPath();
        ctx.arc(Xz(zc), Yz(wc), 5, 0, Math.PI * 2);
        ctx.fill();
        const nameI = containing[0].id.replace('U', 'φ').replace('V', 'ψ');
        const nameJ = containing[1].id.replace('U', 'φ').replace('V', 'ψ');
        label(ctx, `transition ${nameI} ∘ ${nameJ}⁻¹ on the overlap`, box.x0, box.y0 - 10, p.ink, { size: 10, weight: 600 });
        label(ctx, mode === 'four' ? '= ±√(1 − z²): smooth on an open interval' : '= z ± π: a translation, derivative 1', box.x1 + 10, box.y0 + 12, p.ink, { size: 9.5 });
        label(ctx, 'smooth both ways ⇒ C∞-related (Def. 3.4.5)', box.x1 + 10, box.y0 + 26, p.ink, { size: 9.5 });
      } else if (mode === 'single') {
        const z = SINGLE_CHART_ATTEMPT.coord(theta)!;
        const near = Math.min(z, 2 * Math.PI - z) < 0.25;
        label(ctx, 'one chart, no overlap — nothing to be related to', box.x0, box.y0 - 10, p.ink, { size: 10, weight: 600 });
        label(ctx, near ? 'at the seam the coordinate jumps 0 ↔ 6.28:' : 'the domain is all of S¹, but it is not open', box.x1 + 10, box.y0 + 12, near ? p.goal : p.ink, { size: 9.5 });
        label(ctx, near ? 'no neighbourhood maps continuously — not a chart' : 'approach θ = 0 from both sides to see why', box.x1 + 10, box.y0 + 26, near ? p.goal : p.ink, { size: 9.5 });
      } else {
        label(ctx, 'no overlap here — one chart covers the point', box.x0, box.y0 - 10, p.ink, { size: 10, weight: 600 });
      }
      ctx.restore();
    },
    [theta, atlas, mode],
  );

  return (
    <WidgetFrame
      id="w5.3"
      title="Atlas of the Circle"
      teaches="An angle is not a number. An angle is a point of S¹; a number is its coordinate in one chart, and no single chart covers the circle."
      colorKey={['robot']}
      wide
      caption={
        <>
          Choset&rsquo;s four charts of S¹ as teal arcs — the open half-circles U₁…U₄ — each
          with its own number line. The orange point is handed from chart to chart; wherever two
          overlap, both coordinates exist and the transition map between them is plotted at
          lower right: ±√(1 − z²), smooth on an open interval, which is what makes the four an
          atlas. Drag the point. Switch to the two-chart angle atlas that <code>S1</code> actually
          uses, whose transition map is a translation by π. Then try the single-chart attempt
          θ ∈ [0, 2π) and drag through θ = 0: every point is covered, but the coordinate tears.
        </>
      }
    >
      <SimCanvas
        world={{ minX: 0, minY: 0, maxX: 2, maxY: 1 }}
        draw={draw}
        deps={[theta, mode]}
        aspect={2}
        padding={0}
        onPointer={onPointer}
        ariaLabel="A unit circle with coloured arcs marking chart domains and an orange point on it; to the right, one number line per chart shows the point's coordinate where it exists, and a small plot shows the transition map on the current overlap."
      />
      <ControlPanel columns={1}>
        <ButtonRow>
          <ActionButton emphasis={mode === 'four'} onClick={() => setMode('four')}>Choset&rsquo;s four charts</ActionButton>
          <ActionButton emphasis={mode === 'two'} onClick={() => setMode('two')}>two angle charts (problem 3.9)</ActionButton>
          <ActionButton emphasis={mode === 'single'} onClick={() => setMode('single')}>single-chart attempt</ActionButton>
        </ButtonRow>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={() => { setManual(null); sim.toggle(); }}
        onStep={() => { setManual(null); sim.stepOnce(); }}
        onReset={() => { setManual(null); sim.reset(); }}
        seed={sim.seed}
        tick={sim.tick}
      />
    </WidgetFrame>
  );
}
