'use client';

import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { ellipse2, type Mat, type Vec } from '@/lib/prob/linalg';
import { deadReckoning, kalmanStep } from '@/lib/estimate';

/**
 * f15.3 — Hyperplane and Ellipse (Figs. 8.2–8.4 redrawn, hover to compare).
 *
 * Every coordinate here is produced by `lib/estimate` on Choset's §8.2.6 step:
 * the prediction (x̂⁻, P⁻), the measurement line Ω, the Euclidean foot
 * (the simple observer), the Mahalanobis foot (the noiseless-output observer),
 * the most likely output y* and Ω*, and the Kalman update that lands on it.
 * The misconception it kills: "the update moves perpendicular to the
 * measurement". It does — in the metric of P⁻, which is why uncertain
 * directions move more.
 */

type Focus = 'omega' | 'omegaStar' | 'feet' | null;

/** Ω from the step-1 prediction, with the three candidate updates on it. */
function geometry() {
  const { sys, bel } = deadReckoning();
  const y = 2.7;
  const s = kalmanStep(sys, bel, [0], [y]);
  const xm = s.predicted.x;
  const Pm = s.predicted.P;
  // H = [0 1]: Ω is the horizontal line v = y.
  const nu = y - xm[1];
  const euclid: Vec = [xm[0], y]; // Hᵀ(HHᵀ)⁻¹ν with H = [0 1]
  const PHt = [Pm[0][1], Pm[1][1]];
  const HPHt = Pm[1][1];
  const mahal: Vec = [xm[0] + (PHt[0] / HPHt) * nu, xm[1] + (PHt[1] / HPHt) * nu];
  const yStar = xm[1] + (HPHt / (HPHt + sys.W[0][0])) * nu;
  return { xm, Pm, y, yStar, euclid, mahal, post: s.posterior.x, Ppost: s.posterior.P, R: s.R.map((r) => r[0]) };
}

const X0 = 1.6;
const X1 = 6.4;
const V0 = 0.9;
const V1 = 6.5;
const W = 520;
const H = 400;
const PAD = 36;
const px = (x: number) => PAD + ((x - X0) / (X1 - X0)) * (W - 2 * PAD);
const py = (v: number) => H - PAD - ((v - V0) / (V1 - V0)) * (H - 2 * PAD);

function ellipsePath(mean: Vec, P: Mat, k: number): string {
  const e = ellipse2(P, k);
  const pts: string[] = [];
  for (let i = 0; i <= 90; i++) {
    const t = (i / 90) * 2 * Math.PI;
    const ex = e.rx * Math.cos(t);
    const ey = e.ry * Math.sin(t);
    const wx = mean[0] + ex * Math.cos(e.angle) - ey * Math.sin(e.angle);
    const wv = mean[1] + ex * Math.sin(e.angle) + ey * Math.cos(e.angle);
    pts.push(`${i === 0 ? 'M' : 'L'}${px(wx).toFixed(1)},${py(wv).toFixed(1)}`);
  }
  return pts.join(' ') + ' Z';
}

export function HyperplaneEllipse() {
  const [focus, setFocus] = useState<Focus>(null);
  const g = useMemo(geometry, []);
  const dim = (role: 'omega' | 'omegaStar' | 'feet') => (focus === null || focus === role ? 1 : 0.18);

  // The Mahalanobis-equidistant ellipse that just touches Ω: its "radius" is
  // the Mahalanobis length of the correction, ‖Δx‖_M = |ν| / √(H P⁻ Hᵀ).
  const kTouch = Math.abs(g.y - g.xm[1]) / Math.sqrt(g.Pm[1][1]);
  const kTouchStar = Math.abs(g.yStar - g.xm[1]) / Math.sqrt(g.Pm[1][1]);

  return (
    <WidgetFrame
      id="f15.3"
      title="Hyperplane and Ellipse"
      teaches="The update is perpendicular to the measurement line — in the covariance metric. That is why uncertain directions move more."
      colorKey={['prediction', 'measurement', 'posterior', 'truth']}
      caption={
        <>
          Choset&rsquo;s §8.2.6 step in the (x<sub>r</sub>, v<sub>r</sub>) plane. The orange ellipse is the
          predicted belief; the solid green line is Ω = {'{'}x : Hx = y{'}'}, every state that would produce the
          reading y = 2.7 exactly. The gray foot drops straight down — the simple observer of §8.2.2, which
          corrects only velocity. The purple foot is where the <em>Mahalanobis</em>-equidistant ellipse first
          touches Ω: because position and velocity are correlated in P⁻, correcting the velocity drags the
          position with it. With sensor noise the line itself moves to Ω* (dashed) through the most likely output
          y* = {g.yStar.toFixed(2)}, and the Kalman update x̂⁺ = ({g.post[0].toFixed(3)}, {g.post[1].toFixed(2)})
          is the Mahalanobis foot on <em>that</em> line.
        </>
      }
    >
      <div className="flex flex-wrap gap-2 border-b border-fd-border px-3 py-2 font-ui text-xs">
        {(
          [
            ['omega', 'Ω: noiseless reading'],
            ['omegaStar', 'Ω*: most likely output'],
            ['feet', 'Euclidean vs Mahalanobis foot'],
          ] as const
        ).map(([key, text]) => (
          <button
            key={key}
            type="button"
            onMouseEnter={() => setFocus(key)}
            onMouseLeave={() => setFocus(null)}
            onFocus={() => setFocus(key)}
            onBlur={() => setFocus(null)}
            className="rounded-sm border border-fd-border bg-fd-card px-2 py-0.5 hover:bg-fd-accent focus-visible:outline-2 focus-visible:outline-fd-primary"
          >
            {text}
          </button>
        ))}
        <span className="ml-auto self-center text-fd-muted-foreground">hover to isolate</span>
      </div>

      <svg
        viewBox={`0 0 ${W} ${H}`}
        role="img"
        aria-label="A prediction ellipse in the position-velocity plane, a horizontal measurement line, and two candidate corrected points on it: one directly below the prediction, one shifted left along the ellipse's tilt. A dashed second line marks the most likely output and the Kalman update on it."
        className="block w-full"
        style={{ background: 'var(--pr-canvas-bg)' }}
      >
        {/* axes */}
        <line x1={PAD} y1={H - PAD} x2={W - PAD} y2={H - PAD} stroke="var(--pr-wall)" strokeWidth={1} />
        <line x1={PAD} y1={PAD} x2={PAD} y2={H - PAD} stroke="var(--pr-wall)" strokeWidth={1} />
        <text x={W - PAD} y={H - PAD + 16} textAnchor="end" fontSize={11} fill="var(--pr-canvas-ink)" fontFamily="ui-monospace, monospace">
          position x_r
        </text>
        <text x={PAD + 4} y={PAD - 10} fontSize={11} fill="var(--pr-canvas-ink)" fontFamily="ui-monospace, monospace">
          velocity v_r
        </text>
        {[2, 3, 4, 5, 6].map((x) => (
          <text key={`x${x}`} x={px(x)} y={H - PAD + 14} textAnchor="middle" fontSize={9} fill="var(--pr-truth)" fontFamily="ui-monospace, monospace">
            {x}
          </text>
        ))}
        {[1, 2, 3, 4, 5, 6].map((v) => (
          <text key={`v${v}`} x={PAD - 6} y={py(v) + 3} textAnchor="end" fontSize={9} fill="var(--pr-truth)" fontFamily="ui-monospace, monospace">
            {v}
          </text>
        ))}

        {/* prediction: 95% ellipse plus the touching equidistant ellipses */}
        <path d={ellipsePath(g.xm, g.Pm, 2.447)} fill="var(--pr-prediction)" fillOpacity={0.12} stroke="var(--pr-prediction)" strokeWidth={1.75} />
        <g opacity={dim('feet') * dim('omega')}>
          <path d={ellipsePath(g.xm, g.Pm, kTouch)} fill="none" stroke="var(--pr-posterior)" strokeWidth={1} strokeDasharray="3 3" />
        </g>
        <g opacity={dim('omegaStar')}>
          <path d={ellipsePath(g.xm, g.Pm, kTouchStar)} fill="none" stroke="var(--pr-posterior)" strokeWidth={1} strokeDasharray="2 4" />
        </g>
        <circle cx={px(g.xm[0])} cy={py(g.xm[1])} r={4} fill="var(--pr-prediction)" />
        <text x={px(g.xm[0]) + 8} y={py(g.xm[1]) - 8} fontSize={11} fill="var(--pr-prediction)" fontFamily="ui-monospace, monospace">
          x̂(k+1|k) = ({g.xm[0]}, {g.xm[1]})
        </text>

        {/* Ω */}
        <g opacity={dim('omega') * Math.max(dim('feet'), 0.5)}>
          <line x1={PAD} y1={py(g.y)} x2={W - PAD} y2={py(g.y)} stroke="var(--pr-measurement)" strokeWidth={2} />
          <text x={W - PAD - 4} y={py(g.y) - 6} textAnchor="end" fontSize={11} fill="var(--pr-measurement)" fontFamily="ui-monospace, monospace">
            Ω = {'{'}x : Hx = y{'}'},  y = {g.y}
          </text>
        </g>

        {/* Ω* */}
        <g opacity={dim('omegaStar')}>
          <line x1={PAD} y1={py(g.yStar)} x2={W - PAD} y2={py(g.yStar)} stroke="var(--pr-measurement)" strokeWidth={2} strokeDasharray="7 4" />
          <text x={PAD + 6} y={py(g.yStar) - 6} fontSize={11} fill="var(--pr-measurement)" fontFamily="ui-monospace, monospace">
            Ω* : Hx = y* = {g.yStar.toFixed(2)}
          </text>
          <line x1={px(g.xm[0])} y1={py(g.xm[1])} x2={px(g.post[0])} y2={py(g.post[1])} stroke="var(--pr-posterior)" strokeWidth={1.5} />
          <circle cx={px(g.post[0])} cy={py(g.post[1])} r={4.5} fill="var(--pr-posterior)" />
          <text x={px(g.post[0]) - 8} y={py(g.post[1]) + 16} textAnchor="end" fontSize={11} fill="var(--pr-posterior)" fontFamily="ui-monospace, monospace">
            x̂(k+1|k+1) = x̂⁻ + Rν
          </text>
        </g>

        {/* the two feet on Ω */}
        <g opacity={dim('feet') * dim('omega')}>
          <line x1={px(g.xm[0])} y1={py(g.xm[1])} x2={px(g.euclid[0])} y2={py(g.euclid[1])} stroke="var(--pr-truth)" strokeWidth={1.5} strokeDasharray="4 3" />
          <circle cx={px(g.euclid[0])} cy={py(g.euclid[1])} r={4.5} fill="none" stroke="var(--pr-truth)" strokeWidth={2} />
          <text x={px(g.euclid[0]) + 8} y={py(g.euclid[1]) + 16} fontSize={11} fill="var(--pr-truth)" fontFamily="ui-monospace, monospace">
            Euclidean foot Hᵀ(HHᵀ)⁻¹ν
          </text>
          <line x1={px(g.xm[0])} y1={py(g.xm[1])} x2={px(g.mahal[0])} y2={py(g.mahal[1])} stroke="var(--pr-posterior)" strokeWidth={1.5} strokeDasharray="4 3" />
          <circle cx={px(g.mahal[0])} cy={py(g.mahal[1])} r={4.5} fill="none" stroke="var(--pr-posterior)" strokeWidth={2} />
          <text x={px(g.mahal[0]) - 8} y={py(g.mahal[1]) - 10} textAnchor="end" fontSize={11} fill="var(--pr-posterior)" fontFamily="ui-monospace, monospace">
            Mahalanobis foot P⁻Hᵀ(HP⁻Hᵀ)⁻¹ν = ({g.mahal[0].toFixed(2)}, {g.mahal[1].toFixed(2)})
          </text>
        </g>
      </svg>
    </WidgetFrame>
  );
}
