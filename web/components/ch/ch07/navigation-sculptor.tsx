'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Vec } from '@/lib/potential/field';
import {
  FIVE_DISC_GOAL,
  FIVE_DISC_WORLD,
  FIVE_STAR_WORLD,
  NavigationFunction,
  criticalPoints,
  hLambda,
  hLambdaInverse,
  hLambdaJacobian,
  inFreeSpace,
  inStarFreeSpace,
  starBoundary,
  starNavigationFunction,
  suitableLambda,
  type CriticalPoint,
} from '@/lib/potential/navigation';
import { arrow, diamond, drawField, fieldImage, fmt, marker } from './shared';

/**
 * w7.2 — the Navigation Function Sculptor.
 *
 * The five-disc sphere world of Choset Fig. 4.14 with the contours of
 * φ = d² / [d^{2κ} + β]^{1/κ}. The headline slider is κ: at small κ the field
 * has spurious minima (red dots that are not the goal); as κ grows they slide
 * toward their obstacles, turn into saddles (hollow diamonds) and vanish, until
 * at κ = 10 there is exactly one minimum. Beads released from seeded starts
 * descend the normalized gradient and show which basin they land in.
 *
 * Star mode replaces each disc by a star-shaped blob and draws φ ∘ h_λ, with
 * h_λ as a displacement field; the critical points of the model world are
 * carried back through h_λ⁻¹ one-to-one. The λ slider is a multiple of the λ₀
 * `suitableLambda` computes — far below it h_λ folds (det J_h < 0 somewhere).
 */

const R = FIVE_DISC_WORLD.r0;
const BOUNDS = { minX: -R, minY: -R, maxX: R, maxY: R };
const FIELD_N = 110;
const N_BEADS = 7;

interface Bead {
  q: Vec;
  trail: Vec[];
  done: 'goal' | 'stuck' | null;
}
interface State {
  beads: Bead[];
  idle: number;
}

export function NavigationSculptor() {
  const [kappa, setKappa] = useState(4);
  const [starMode, setStarMode] = useState(false);
  const [logMult, setLogMult] = useState(0);
  const [goal, setGoal] = useState<Vec>(FIVE_DISC_GOAL);
  const [showArrows, setShowArrows] = useState(true);
  const dragging = useRef(false);

  const lambda0 = useMemo(() => (starMode ? suitableLambda(FIVE_STAR_WORLD, goal, kappa, 32) : 1), [starMode, goal, kappa]);
  const lambda = lambda0 * Math.pow(10, logMult);
  const h = useMemo(() => (starMode ? hLambda(FIVE_STAR_WORLD, goal, kappa, lambda) : null), [starMode, goal, kappa, lambda]);
  const phi = useMemo(() => (starMode ? starNavigationFunction(FIVE_STAR_WORLD, goal, kappa, lambda) : new NavigationFunction(FIVE_DISC_WORLD, goal, kappa)), [starMode, goal, kappa, lambda]);
  const free = useCallback((q: Vec) => (starMode ? inStarFreeSpace(FIVE_STAR_WORLD, q) : inFreeSpace(FIVE_DISC_WORLD, q)), [starMode]);

  // Critical points on the model sphere world, carried back by h_λ⁻¹ in star mode.
  const crit = useMemo(() => {
    const model = criticalPoints(FIVE_DISC_WORLD, goal, kappa, { seedsPerAxis: 24 });
    if (!h) return { ...model, lost: 0 };
    const points: CriticalPoint[] = [];
    let lost = 0;
    for (const c of model.points) {
      const q = hLambdaInverse(FIVE_STAR_WORLD, h, c.q);
      if (q) points.push({ ...c, q });
      else lost++;
    }
    return { points, minima: points.filter((c) => c.type === 'minimum').length, saddles: points.filter((c) => c.type === 'saddle').length, maxima: points.filter((c) => c.type === 'maximum').length, lost };
  }, [goal, kappa, h]);

  // The diffeomorphism test on a coarse lattice (star mode only).
  const minDet = useMemo(() => {
    if (!h) return null;
    let m = Infinity;
    const N = 22;
    for (let a = 0; a < N; a++) {
      for (let b = 0; b < N; b++) {
        const q: Vec = [-R + ((a + 0.5) / N) * 2 * R, -R + ((b + 0.5) / N) * 2 * R];
        if (!inStarFreeSpace(FIVE_STAR_WORLD, q)) continue;
        m = Math.min(m, hLambdaJacobian(h, q).det);
      }
    }
    return m;
  }, [h]);

  const field = useMemo(() => {
    const vals = new Float32Array(FIELD_N * FIELD_N);
    const cs = (2 * R) / FIELD_N;
    for (let j = 0; j < FIELD_N; j++) {
      for (let i = 0; i < FIELD_N; i++) {
        const q: Vec = [-R + (i + 0.5) * cs, -R + (j + 0.5) * cs];
        vals[j * FIELD_N + i] = free(q) ? phi.value(q) : -1;
      }
    }
    return vals;
  }, [phi, free]);
  const imgCache = useRef<{ key: string; img: HTMLCanvasElement | null } | null>(null);

  const init = useCallback(
    (seed: number): State => {
      const rng = new Rng(seed);
      const beads: Bead[] = [];
      while (beads.length < N_BEADS) {
        const a = rng.uniform(0, 2 * Math.PI);
        const r = rng.uniform(1.0, R - 0.4);
        const q: Vec = [r * Math.cos(a), r * Math.sin(a)];
        if (free(q) && Math.hypot(q[0] - goal[0], q[1] - goal[1]) > 0.8) beads.push({ q, trail: [q], done: null });
      }
      return { beads, idle: 0 };
    },
    [free, goal],
  );

  const step = useCallback(
    (s: State): State => {
      if (s.beads.every((b) => b.done)) return s.idle < 40 ? { ...s, idle: s.idle + 1 } : init(Math.floor(Math.random() * 1e5));
      const beads = s.beads.map((b) => {
        if (b.done) return b;
        // Normalized descent: φ is flat near the goal and far from it (the plateau problem), so a
        // fixed-α step would crawl there and leap in between. The direction is still −∇φ.
        const g = phi.gradient(b.q);
        const gn = Math.hypot(g[0], g[1]);
        if (Math.hypot(b.q[0] - goal[0], b.q[1] - goal[1]) < 0.06) return { ...b, done: 'goal' as const };
        if (gn < 1e-13 || b.trail.length > 1500) return { ...b, done: 'stuck' as const };
        const sLen = 0.035;
        const next: Vec = [b.q[0] - (g[0] / gn) * sLen, b.q[1] - (g[1] / gn) * sLen];
        if (!free(next)) return { ...b, done: 'stuck' as const };
        // Oscillation across a minimum: the last few moves cancel. Declare it stuck.
        const t = b.trail;
        if (t.length > 30) {
          const back = t[t.length - 30];
          if (Math.hypot(next[0] - back[0], next[1] - back[1]) < 0.5 * sLen) return { ...b, done: 'stuck' as const };
        }
        return { ...b, q: next, trail: [...t, next] };
      });
      return { ...s, beads };
    },
    [phi, free, goal, init],
  );

  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 72 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [init, phi]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') {
        dragging.current = false;
        return;
      }
      if (phase === 'down') {
        dragging.current = Math.hypot(w[0] - goal[0], w[1] - goal[1]) < 0.4;
        return;
      }
      if (dragging.current) {
        const q: Vec = [w[0], w[1]];
        const ok = inFreeSpace(FIVE_DISC_WORLD, q) && inStarFreeSpace(FIVE_STAR_WORLD, q) && Math.hypot(q[0], q[1]) < R - 0.3;
        if (ok) setGoal(q);
      }
    },
    [goal],
  );

  const s = sim.state;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const key = `${p.ink}:${kappa}:${starMode}:${logMult}:${goal[0]},${goal[1]}`;
      if (!imgCache.current || imgCache.current.key !== key) {
        // Banded alpha: 16 bands of φ with alternating weight read as contour rings.
        imgCache.current = { key, img: fieldImage(field, FIELD_N, FIELD_N, p.ink, (f) => (f < 0 ? 0 : 0.04 + 0.42 * Math.pow(f, 0.6) + (Math.floor(f * 16) % 2 ? 0.1 : 0))) };
      }
      // Clip to the bounding circle.
      ctx.save();
      ctx.beginPath();
      ctx.arc(sx(v, 0), sy(v, 0), sl(v, R), 0, Math.PI * 2);
      ctx.clip();
      if (imgCache.current.img) drawField(ctx, v, imgCache.current.img, BOUNDS, true);
      ctx.restore();
      // Obstacles.
      ctx.save();
      ctx.fillStyle = p.obstacle;
      if (starMode) {
        for (const o of FIVE_STAR_WORLD.obstacles) {
          ctx.beginPath();
          starBoundary(o, 72).forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
          ctx.closePath();
          ctx.fill();
          // The model disc, dashed, inscribed.
          ctx.save();
          ctx.strokeStyle = p.bg;
          ctx.setLineDash([3, 3]);
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.arc(sx(v, o.c.x), sy(v, o.c.y), sl(v, o.r), 0, Math.PI * 2);
          ctx.stroke();
          ctx.restore();
        }
      } else {
        for (const o of FIVE_DISC_WORLD.obstacles) {
          ctx.beginPath();
          ctx.arc(sx(v, o.c.x), sy(v, o.c.y), sl(v, o.r), 0, Math.PI * 2);
          ctx.fill();
        }
      }
      ctx.strokeStyle = p.obstacle;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(sx(v, 0), sy(v, 0), sl(v, R), 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
      // h_λ as a displacement field.
      if (h && showArrows) {
        const N = 16;
        for (let a = 0; a < N; a++) {
          for (let b = 0; b < N; b++) {
            const q: Vec = [-R + ((a + 0.5) / N) * 2 * R, -R + ((b + 0.5) / N) * 2 * R];
            if (!inStarFreeSpace(FIVE_STAR_WORLD, q) || Math.hypot(q[0], q[1]) > R) continue;
            const hq = h(q);
            const dx = hq[0] - q[0];
            const dy = hq[1] - q[1];
            if (Math.hypot(dx, dy) < 0.01) continue;
            arrow(ctx, v, q[0], q[1], dx, dy, p.cobstacle, 1.2, 4);
          }
        }
      }
      // Beads.
      for (const b of s.beads) {
        ctx.save();
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 1.6;
        ctx.globalAlpha = 0.8;
        ctx.beginPath();
        b.trail.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt[0]), sy(v, pt[1])) : ctx.lineTo(sx(v, pt[0]), sy(v, pt[1]))));
        ctx.stroke();
        ctx.restore();
        marker(ctx, sx(v, b.trail[0][0]), sy(v, b.trail[0][1]), 3.5, p.start);
        marker(ctx, sx(v, b.q[0]), sy(v, b.q[1]), 5, b.done === 'stuck' ? p.goal : p.robot, p.bg);
      }
      // Critical points: minima red (the goal with a ring), saddles hollow diamonds, maxima ink.
      for (const c of crit.points) {
        const px = sx(v, c.q[0]);
        const py = sy(v, c.q[1]);
        if (c.type === 'minimum') marker(ctx, px, py, 6, p.goal, p.bg);
        else if (c.type === 'saddle') diamond(ctx, px, py, 6, p.ink);
        else marker(ctx, px, py, 5, p.ink, p.bg);
      }
      marker(ctx, sx(v, goal[0]), sy(v, goal[1]), 8, p.goal, p.bg, 2.5);
      const stuck = s.beads.filter((b) => b.done === 'stuck').length;
      label(ctx, `κ = ${kappa}   minima ${crit.minima}   saddles ${crit.saddles}   maxima ${crit.maxima}${crit.minima > 1 ? `   — ${crit.minima - 1} spurious` : '   — a navigation function'}`, 10, 14, crit.minima > 1 ? p.goal : p.ink, { size: 10.5, weight: 600 });
      if (starMode) label(ctx, `star mode: φ ∘ h_λ,  λ = 10^${logMult} · λ₀,  min det J_h on a lattice = ${minDet === null ? '…' : fmt(minDet, 2)}${minDet !== null && minDet <= 0 ? '  — folds: not a diffeomorphism' : ''}${crit.lost ? `, ${crit.lost} critical point(s) not recovered by h_λ⁻¹` : ''}`, 10, 30, minDet !== null && minDet <= 0 ? p.goal : p.ink, { size: 10 });
      label(ctx, `${stuck} of ${s.beads.length} beads stuck away from the goal · drag the goal`, 10, v.height - 12, p.ink, { size: 9 });
    },
    [s, field, kappa, starMode, logMult, goal, h, showArrows, crit, minDet],
  );

  return (
    <WidgetFrame
      id="w7.2"
      title="Navigation Function Sculptor"
      teaches="Potentials do not have to have spurious minima: on a sphere world φ has exactly one for large enough κ, and a diffeomorphism carries that onto star worlds."
      colorKey={['goal', 'obstacle', 'cobstacle', 'robot', 'start']}
      wide
      caption={
        <>
          Contour bands of Choset&rsquo;s eq. (4.13) on the five-disc sphere world. Red dots are
          minima (the ringed one is the goal), hollow diamonds are saddles, found by Newton polishing
          from a lattice and classified by the Hessian. Slide κ up from 2: the spurious minima drift
          toward their discs, become saddles, and are gone by κ = 10 — the chapter&rsquo;s check pins
          that count. Orange beads descend the normalized gradient; the ones that stop red landed in
          a spurious basin. In star mode each disc grows lobes, the amber arrows show q ↦ h<sub>λ</sub>(q),
          and the critical points are the model world&rsquo;s pulled back through h<sub>λ</sub><sup>−1</sup>;
          drag λ far below λ₀ and the readout reports det J<sub>h</sub> &lt; 0 — the map folds and the
          correspondence breaks.
        </>
      }
    >
      <SimCanvas world={BOUNDS} draw={draw} deps={[sim.tick, s, kappa, starMode, logMult, goal, showArrows]} aspect={16 / 10} padding={0.2} onPointer={onPointer} ariaLabel="A disc-shaped world with five slate obstacles and shaded contour bands of a navigation function; red dots mark minima, diamonds mark saddles, orange beads roll downhill." />
      <ControlPanel columns={3} title="Headline: κ">
        <Slider label="κ — the sharpening exponent" role="goal" value={kappa} min={2} max={12} step={1} onChange={(v) => setKappa(Math.round(v))} />
        <Toggle label="Star mode: star-shaped obstacles and φ ∘ h_λ (§4.6.2)" role="cobstacle" checked={starMode} onChange={setStarMode} />
        <Toggle label="Draw h_λ as a displacement field" role="cobstacle" checked={showArrows} onChange={setShowArrows} />
      </ControlPanel>
      {starMode ? (
        <ControlPanel columns={2}>
          <Slider label="log₁₀(λ / λ₀) — the switch sharpness" role="cobstacle" value={logMult} min={-6} max={2} step={0.5} format={(v) => `10^${v}`} onChange={setLogMult} />
          <div className="font-mono text-[0.72rem] text-fd-muted-foreground tabular-nums">
            λ₀ = {lambda0.toExponential(2)} from <code>suitableLambda</code>; h<sub>λ</sub> maps every star boundary onto its inscribed circle for any λ — only the interior depends on it.
          </div>
        </ControlPanel>
      ) : null}
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
