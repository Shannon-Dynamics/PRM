'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawRobot, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { MICRO_Q, MICRO_SITES, homeLaw, homeToMeetPoint, pointSitesQuery, traceLaw, vdot, vnorm, type LawStep, type Vec } from '@/lib/roadmap/gvg';
import { arrow, drawField, fieldImage, marker } from '@/components/ch/ch07/shared';
import { polyline } from './shared';

/**
 * w9.3 — the Lyapunov Funnel (Choset §5.3.3, eq. 5.5).
 *
 * The chapter's micro-world: two point obstacles at (0, 0) and (4, 0), whose
 * GVG is the bisector x = 2, and Rusty at (2.5, 1). The tracing law
 * q̇ = αv + β(DG)†G is drawn as a field of arrows; Γ = ½GᵀG is shaded as a
 * heightfield — a trough along the bisector — and Rusty slides down it while
 * advancing along the edge. The sparkline plots Γ(t) against Γ₀e^{2βt},
 * which is what Γ̇ = βGᵀG = 2βΓ integrates to; the readout compares the
 * correction's component along the sites' axis with minus Rusty's signed
 * distance to the bisector (Exercise 1: they agree exactly for point sites,
 * at β = −1). With β > 0 the robot peels away. Homing mode adds a third site:
 * Null(DG) is empty, the tangent term vanishes, and the same law runs Newton
 * onto the meet point. The controller never sees an equation of the GVG —
 * only G and DG, which a range ring measures.
 */

const BOUNDS = { minX: -1.5, minY: -3, maxX: 5.5, maxY: 4 };
const FIELD_N = 112;
const DT = 0.02;
const SUBSTEPS = 2;
const THIRD: Vec = [2, 3.2];
const MAX_T = 14;

interface State {
  q: Vec;
  heading: Vec;
  trail: Vec[];
  gamma: number[];
  t: number;
  last: LawStep | null;
  rng: Rng;
  settled: number;
  /** 'start' holds the opening position for a moment so its numbers can be read. */
  stop: 'start' | 'running' | 'left' | 'singular' | 'settled' | 'timeout';
}

function inView(q: Vec) {
  return q[0] > BOUNDS.minX - 0.2 && q[0] < BOUNDS.maxX + 0.2 && q[1] > BOUNDS.minY - 0.2 && q[1] < BOUNDS.maxY + 0.2;
}

export function LyapunovFunnel() {
  const [beta, setBeta] = useState(-1);
  const [alpha, setAlpha] = useState(1);
  const [homing, setHoming] = useState(false);
  const [showField, setShowField] = useState(true);
  const [sites, setSites] = useState<Vec[]>(MICRO_SITES.map((s) => s.slice()));
  const [third, setThird] = useState<Vec>(THIRD);
  const dragging = useRef<number | null>(null);
  const placed = useRef<Vec | null>(null);

  const allSites = useMemo(() => (homing ? [...sites, third] : sites), [homing, sites, third]);
  // Every site enters G (two for an edge, three for homing), so the readings are taken in the sites'
  // labelled order — G = d₁ − d₂ with site 1 at the origin, the sign the micro-example prints. The
  // law itself (q̇, Γ, Γ̇) does not depend on the order.
  const query = useMemo(() => {
    const q = pointSitesQuery(allSites);
    return (x: Vec) => q(x).sort((a, b) => a.index - b.index);
  }, [allSites]);
  const params = useRef({ beta, alpha, homing, query });
  params.current = { beta, alpha, homing, query };

  /** One evaluation of the law at q: edge tracing with two sites, homing with three. */
  const law = useCallback((q: Vec, heading: Vec, a: number, b: number, home: boolean, qy = query): LawStep => {
    const near = qy(q);
    return home ? homeLaw(near, b, 2) : traceLaw(near, a, b, heading);
  }, [query]);

  const fresh = useCallback((rng: Rng, q: Vec, heading: Vec): State => {
    const { alpha: a, beta: b, homing: h, query: qy } = params.current;
    const last = law(q, heading, a, b, h, qy);
    return { q: q.slice(), heading, trail: [q.slice()], gamma: [last.gamma], t: 0, last, rng, settled: 0, stop: 'start' };
  }, [law]);

  const init = useCallback(
    (seed: number): State => {
      const rng = new Rng(seed);
      // The micro-example's start on the book's seed; elsewhere a seeded point of the window.
      const q0 = placed.current ?? (seed === 9 ? MICRO_Q.slice() : [rng.uniform(-0.8, 4.8), rng.uniform(-2.2, 2.4)]);
      return fresh(rng, q0, [0, rng.uniform(0, 1) < 0.5 && seed !== 9 ? -1 : 1]);
    },
    [fresh],
  );

  const step = useCallback(
    (s: State): State => {
      const { alpha: a, beta: b, homing: h, query: qy } = params.current;
      if (s.stop === 'start') {
        if (s.settled++ < 40) return { ...s, last: law(s.q, s.heading, a, b, h, qy) };
        s = { ...s, stop: 'running', settled: 0 };
      }
      if (s.stop !== 'running') {
        // Linger, then respawn at a seeded random point: the funnel works from anywhere.
        if (s.settled++ < 45) return { ...s };
        placed.current = null;
        return fresh(s.rng, [s.rng.uniform(-0.8, 4.8), s.rng.uniform(-2.2, 2.4)], [0, s.rng.uniform(0, 1) < 0.5 ? -1 : 1]);
      }
      let { q, heading } = s;
      let last = s.last;
      let stop: State['stop'] = 'running';
      const trail = s.trail;
      const gamma = s.gamma;
      for (let k = 0; k < SUBSTEPS; k++) {
        last = law(q, heading, a, b, h, qy);
        if (last.singular) {
          stop = 'singular';
          break;
        }
        q = [q[0] + DT * last.qdot[0], q[1] + DT * last.qdot[1]];
        if (vnorm(last.tangent) > 0) heading = last.tangent.map((x) => x / vnorm(last!.tangent));
      }
      const after = law(q, heading, a, b, h, qy);
      trail.push(q.slice());
      if (trail.length > 1200) trail.shift();
      gamma.push(after.gamma);
      if (gamma.length > 600) gamma.shift();
      const t = s.t + SUBSTEPS * DT;
      if (stop === 'running') {
        if (!inView(q)) stop = 'left';
        else if (h && after.gamma < 1e-9) stop = 'settled';
        else if (t > MAX_T) stop = 'timeout';
      }
      return { ...s, q, heading, trail, gamma, t, last: after, stop, settled: 0 };
    },
    [law, fresh],
  );

  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 9 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  // Changing the world restarts the run from the same start (the gains stay live).
  useEffect(() => {
    resetRef.current();
  }, [homing, sites, third]);

  const meet = useMemo(() => (homing ? homeToMeetPoint(query, [2, 1]) : null), [homing, query]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') {
        dragging.current = null;
        return;
      }
      const list = homing ? [...sites, third] : sites;
      if (phase === 'down') {
        const hit = list.findIndex((c) => Math.hypot(c[0] - w[0], c[1] - w[1]) < 0.3);
        if (hit >= 0) {
          dragging.current = hit;
          return;
        }
        // Anywhere else: put Rusty there.
        placed.current = [w[0], w[1]];
        sim.setState((s) => fresh(s.rng, [w[0], w[1]], [0, 1]));
        return;
      }
      const d = dragging.current;
      if (d === null) return;
      const c: Vec = [Math.max(BOUNDS.minX, Math.min(BOUNDS.maxX, w[0])), Math.max(BOUNDS.minY, Math.min(BOUNDS.maxY, w[1]))];
      if (d < sites.length) setSites((prev) => prev.map((x, i) => (i === d ? c : x)));
      else setThird(c);
    },
    [homing, sites, third, sim, fresh],
  );

  // The funnel Γ = ½|G|² on a grid (G from the k nearest sites, as the law sees them).
  const field = useMemo(() => {
    const vals = new Float32Array(FIELD_N * FIELD_N);
    const cw = (BOUNDS.maxX - BOUNDS.minX) / FIELD_N;
    const ch = (BOUNDS.maxY - BOUNDS.minY) / FIELD_N;
    let max = 0;
    for (let j = 0; j < FIELD_N; j++) {
      for (let i = 0; i < FIELD_N; i++) {
        const q = [BOUNDS.minX + (i + 0.5) * cw, BOUNDS.minY + (j + 0.5) * ch];
        const near = query(q);
        let g2 = 0;
        for (let k = 1; k < near.length; k++) g2 += (near[0].d - near[k].d) ** 2;
        vals[j * FIELD_N + i] = 0.5 * g2;
        max = Math.max(max, 0.5 * g2);
      }
    }
    return { vals, max };
  }, [query, homing]);
  const imgCache = useRef<{ key: string; img: HTMLCanvasElement | null } | null>(null);

  const s = sim.state;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const key = `${p.ink}:${allSites.map((c) => c.join(',')).join(';')}:${homing}`;
      if (!imgCache.current || imgCache.current.key !== key) {
        imgCache.current = { key, img: fieldImage(field.vals, FIELD_N, FIELD_N, p.ink, (g) => 0.03 + 0.42 * (1 - Math.exp(-g / 0.5))) };
      }
      if (imgCache.current.img) drawField(ctx, v, imgCache.current.img, BOUNDS, true);
      drawGrid(ctx, v, p, 1);

      // The GVG: the bisector of the two sites (edge mode) or the meet point (homing).
      if (!homing) {
        const [a, b] = sites;
        const m = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        const u = [-(b[1] - a[1]), b[0] - a[0]];
        const L = Math.hypot(u[0], u[1]) || 1;
        const r = 12;
        polyline(ctx, v, [[m[0] - (r * u[0]) / L, m[1] - (r * u[1]) / L], [m[0] + (r * u[0]) / L, m[1] + (r * u[1]) / L]], p.graph, 2.2, 0.9);
      } else if (meet) {
        marker(ctx, sx(v, meet.q[0]), sy(v, meet.q[1]), 6, p.bg, p.graph, 2.5);
      }

      // The law as a field: q̇ at a lattice, unit-normalized so the direction reads.
      if (showField) {
        for (let x = BOUNDS.minX + 0.5; x < BOUNDS.maxX; x += 0.5) {
          for (let y = BOUNDS.minY + 0.5; y < BOUNDS.maxY; y += 0.5) {
            const st = law([x, y], [0, 1], alpha, beta, homing);
            if (st.singular) continue;
            const n = vnorm(st.qdot);
            if (n < 1e-9) continue;
            const len = 0.32 * Math.min(1, n / 1.2);
            arrow(ctx, v, x, y, (len * st.qdot[0]) / n, (len * st.qdot[1]) / n, p.accent, 1.1, 4);
          }
        }
      }

      for (const c of allSites) marker(ctx, sx(v, c[0]), sy(v, c[1]), 7, p.obstacle);

      polyline(ctx, v, s.trail, p.robot, 2, 0.85);
      const st = s.last;
      if (st && !st.singular) {
        // The two terms of the law at Rusty: tangent αv (along the edge) and correction β(DG)†G (toward it).
        arrow(ctx, v, s.q[0], s.q[1], st.tangent[0] * 0.8, st.tangent[1] * 0.8, p.graph, 2, 7);
        arrow(ctx, v, s.q[0], s.q[1], st.correction[0] * 0.8, st.correction[1] * 0.8, p.path, 2, 7);
        // Rays to the closest points on the k nearest sites: the readings G is made of.
        const near = query(s.q);
        for (const nn of near) if (nn.closest) polyline(ctx, v, [s.q, nn.closest], p.robot, 1, 0.35, true);
      }
      drawRobot(ctx, v, { x: s.q[0], y: s.q[1], theta: Math.atan2(s.heading[1], s.heading[0]) }, p.robot, 0.28);

      // Readouts.
      const G = st?.G ?? [];
      label(ctx, `q = (${s.q[0].toFixed(3)}, ${s.q[1].toFixed(3)}) · G = ${G.map((g) => g.toFixed(4)).join(', ')} · Γ = ${(st?.gamma ?? 0).toFixed(4)} · Γ̇ = βGᵀG = ${(st?.dGamma ?? 0).toFixed(4)}`, 10, 14, p.ink, { size: 10, weight: 600 });
      if (st && !homing && !st.singular) {
        const [a, b] = sites;
        const L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
        const u = [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
        const off = (s.q[0] - (a[0] + b[0]) / 2) * u[0] + (s.q[1] - (a[1] + b[1]) / 2) * u[1];
        label(ctx, `correction · û = ${vdot(st.correction, u).toFixed(4)}   β × (signed distance to the bisector) = ${(beta * off).toFixed(4)}`, 10, 28, p.ink, { size: 9.5 });
      } else if (homing && meet) {
        label(ctx, `meet point (${meet.q[0].toFixed(3)}, ${meet.q[1].toFixed(3)}) · ‖q − meet‖ = ${Math.hypot(s.q[0] - meet.q[0], s.q[1] - meet.q[1]).toExponential(2)}`, 10, 28, p.ink, { size: 9.5 });
      }
      const msg = s.stop === 'start' ? 't = 0 — the law at the starting point' : s.stop === 'left' ? 'left the window: respawning' : s.stop === 'singular' ? 'DG DGᵀ singular here (∇d₁ = ∇d₂): the law is undefined' : s.stop === 'settled' ? 'on the meet point' : s.stop === 'timeout' ? 'respawning' : '';
      if (msg) label(ctx, msg, 10, v.height - 12, p.ink, { size: 9.5, weight: 600 });
      else label(ctx, 'click to place Rusty · drag a site', 10, v.height - 12, p.ink, { size: 9 });
    },
    [s, field, allSites, sites, homing, meet, showField, law, alpha, beta, query],
  );

  const drawSpark = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const W = v.width;
      const H = v.height;
      // Left: log Γ(t) and the prediction Γ₀ e^{2βt}.
      const g = s.gamma;
      const half = W * 0.55;
      const lo = -12;
      const hi = 1;
      const Y = (val: number) => H - 8 - ((Math.max(lo, Math.log10(Math.max(val, 1e-300))) - lo) / (hi - lo)) * (H - 22);
      const X = (i: number) => 8 + (i / Math.max(1, 299)) * (half - 16);
      const n = Math.min(g.length, 300);
      const g0 = g[g.length - n] ?? 0;
      ctx.save();
      ctx.strokeStyle = p.ink;
      ctx.globalAlpha = 0.55;
      ctx.setLineDash([4, 3]);
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const pred = g0 * Math.exp(2 * beta * i * SUBSTEPS * DT);
        if (i === 0) ctx.moveTo(X(i), Y(pred));
        else ctx.lineTo(X(i), Y(pred));
      }
      ctx.stroke();
      ctx.restore();
      ctx.save();
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const val = g[g.length - n + i];
        if (i === 0) ctx.moveTo(X(i), Y(val));
        else ctx.lineTo(X(i), Y(val));
      }
      ctx.stroke();
      ctx.restore();
      label(ctx, `log₁₀ Γ(t) (orange) vs Γ₀e^{2βt} (dashed)`, 8, 9, p.ink, { size: 9 });

      // Right: the funnel's cross-section through Rusty, perpendicular to the edge (along the sites' axis).
      const x0 = half + 10;
      const x1 = W - 8;
      const [a, b] = sites;
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
      const u = [(b[0] - a[0]) / L, (b[1] - a[1]) / L];
      const off = (s.q[0] - (a[0] + b[0]) / 2) * u[0] + (s.q[1] - (a[1] + b[1]) / 2) * u[1];
      const span = 3;
      const prof: number[] = [];
      for (let i = 0; i <= 80; i++) {
        const t = -span + (2 * span * i) / 80 - off;
        const q = [s.q[0] + t * u[0], s.q[1] + t * u[1]];
        const near = query(q);
        let g2 = 0;
        for (let k = 1; k < near.length; k++) g2 += (near[0].d - near[k].d) ** 2;
        prof.push(0.5 * g2);
      }
      const pmax = Math.max(1e-6, ...prof);
      const PX = (i: number) => x0 + (i / 80) * (x1 - x0);
      const PY = (val: number) => H - 8 - (val / pmax) * (H - 24);
      ctx.save();
      ctx.strokeStyle = p.ink;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      prof.forEach((val, i) => (i === 0 ? ctx.moveTo(PX(i), PY(val)) : ctx.lineTo(PX(i), PY(val))));
      ctx.stroke();
      ctx.restore();
      const bx = x0 + ((off + span) / (2 * span)) * (x1 - x0);
      if (bx >= x0 && bx <= x1) marker(ctx, bx, PY(s.last?.gamma ?? 0), 4.5, p.robot);
      label(ctx, homing ? 'Γ across the site axis, through Rusty' : 'the funnel: Γ across the edge, through Rusty', x0, 9, p.ink, { size: 9 });
    },
    [s, beta, sites, query, homing],
  );

  return (
    <WidgetFrame
      id="w9.3"
      title="Lyapunov Funnel"
      teaches="The controller does not need the GVG's equation — only G and DG, both read off the range ring — and Γ̇ = βGᵀG makes the edge attract."
      colorKey={['obstacle', 'graph', 'path', 'robot']}
      caption={
        <>
          Two point obstacles (slate) and their GVG edge, the bisector (blue). The shading is the Lyapunov
          function Γ = ½G² — dark far from the edge, clear on it — and the teal arrows are the law q̇ =
          αv + β(DG)†G everywhere. At Rusty the law splits into its two terms: the blue arrow αv along the
          edge, the purple arrow β(DG)†G toward it. On the book&rsquo;s seed Rusty starts at (2.5, 1),
          where the readout prints the micro-example: G = 0.8898, Γ = 0.3959, Γ̇ = −0.7918. The sparkline
          shows Γ(t) on the dashed curve Γ₀·exp(2βt) — what Γ̇ = 2βΓ integrates to — and the right-hand
          plot is the funnel&rsquo;s cross-section with Rusty sliding down it. Push β above zero and it
          climbs out. Drag a site: the bisector moves, and the readout&rsquo;s two numbers stay equal
          (Exercise 1). Switch on homing and a third site appears: the tangent term vanishes and the same
          law converges on the meet point.
        </>
      }
    >
      <SimCanvas world={BOUNDS} draw={draw} deps={[sim.tick, draw]} aspect={1.1} padding={0} onPointer={onPointer} cursor="crosshair" ariaLabel="Two point obstacles with their bisector in blue, a shaded trough of the Lyapunov function along it, arrows of the tracing law, and an orange robot sliding onto the bisector while moving along it." />
      <SimCanvas world={{ minX: 0, minY: 0, maxX: 1, maxY: 1 }} draw={drawSpark} deps={[sim.tick, drawSpark]} aspect={4.2} padding={0} ariaLabel="Left: the Lyapunov function against time on a log scale with its exponential prediction. Right: the funnel's cross-section with the robot on it." />
      <ControlPanel columns={2} title="Headline: correction gain β">
        <Slider label="β (1/s)" role="path" value={beta} min={-5} max={1} step={0.1} onChange={setBeta} help="β < 0 makes Γ decrease; β > 0 makes the edge repel." />
        <Slider label="α (speed along the edge, m/s)" role="graph" value={alpha} min={0} max={2} step={0.1} unit="m/s" onChange={setAlpha} />
      </ControlPanel>
      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Homing · layers · reset the sites</summary>
        <ControlPanel columns={3}>
          <div className="flex flex-col gap-1.5">
            <Toggle label="homing: add a third site" role="obstacle" checked={homing} onChange={setHoming} />
            <Toggle label="law as a field" checked={showField} onChange={setShowField} />
          </div>
          <ButtonRow>
            <ActionButton
              onClick={() => {
                setSites(MICRO_SITES.map((c) => c.slice()));
                setThird(THIRD);
                placed.current = MICRO_Q.slice();
                sim.reset();
              }}
            >
              micro-example: sites (0,0), (4,0); Rusty at (2.5, 1)
            </ActionButton>
          </ButtonRow>
        </ControlPanel>
      </details>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => {
        placed.current = null;
        sim.reseed();
      }} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
