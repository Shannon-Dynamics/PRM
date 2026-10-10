'use client';

import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { ControlPanel, Slider } from '@/components/sim/controls';
import { Rng } from '@/lib/prob/rng';
import { gammaStarKf2011, gammaStarSolovey2020, kN, rN } from '@/lib/sampling';

/**
 * f13.4 — The shrinking tiling.
 *
 * A path of length L in the unit square tiled by balls of radius r_n/4, with n
 * uniform samples dropped on it. As n grows the balls shrink like
 * (log n / n)^{1/2}, their number grows like 1/r_n, and yet the expected
 * number of empty balls goes to zero: n · μ(ball) ∝ γ² log n grows, and a
 * ball is empty with probability about n^{−cγ²}. Slide n and count the red
 * (empty) balls. That the count stays small for large n with γ ≥ γ* — and not
 * with γ below it — is the whole content of the radius.
 */

const SIZE = 380;
const PAD = 16;
const L_PATH = 1.2;

/** An S-shaped path of length ≈ 1.2 in the unit square, as a dense polyline. */
function makePath(): { pts: [number, number][]; cum: number[] } {
  const pts: [number, number][] = [];
  const N = 400;
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    pts.push([0.12 + 0.76 * t, 0.5 + 0.3 * Math.sin(2 * Math.PI * t - Math.PI / 2) * (1 - 0.3 * t)]);
  }
  const cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  // Rescale to exactly L_PATH.
  const s = L_PATH / cum[cum.length - 1];
  const c: [number, number] = [0.5, 0.5];
  return { pts: pts.map(([x, y]) => [c[0] + (x - c[0]) * s, c[1] + (y - c[1]) * s]), cum: cum.map((x) => x * s) };
}

const PATH = makePath();
const px = (x: number) => PAD + x * (SIZE - 2 * PAD);
const py = (y: number) => SIZE - PAD - y * (SIZE - 2 * PAD);

function pointAt(sArc: number): [number, number] {
  const { pts, cum } = PATH;
  let i = 1;
  while (i < cum.length - 1 && cum[i] < sArc) i++;
  const t = (sArc - cum[i - 1]) / Math.max(1e-12, cum[i] - cum[i - 1]);
  return [pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]), pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1])];
}

export function ShrinkingTiling() {
  const [logN, setLogN] = useState(2.7);
  const [mult, setMult] = useState(1.0);
  const n = Math.round(Math.pow(10, logN));
  const gamma = gammaStarSolovey2020(2, 1) * mult;
  const r = rN(n, 2, gamma);
  const ballR = r / 4;

  const model = useMemo(() => {
    const rng = new Rng(1304);
    const samples: [number, number][] = [];
    for (let i = 0; i < n; i++) samples.push([rng.next(), rng.next()]);
    const centers: [number, number][] = [];
    const spacing = ballR; // consecutive centers one radius apart: neighbors share a ball of radius r_n/2
    for (let sArc = 0; sArc <= L_PATH + 1e-9; sArc += spacing) centers.push(pointAt(Math.min(sArc, L_PATH)));
    const empty = centers.map((c) => !samples.some((q) => Math.hypot(q[0] - c[0], q[1] - c[1]) <= ballR));
    return { samples, centers, empty };
  }, [n, ballR]);

  const emptyCount = model.empty.filter(Boolean).length;
  const pEmpty = Math.pow(1 - Math.PI * ballR * ballR, n);
  const expected = model.centers.length * pEmpty;

  return (
    <WidgetFrame
      id="f13.4"
      title="The shrinking tiling"
      teaches="Why r_n ∝ (log n / n)^{1/d}: the balls must shrink so that each node has only O(log n) neighbors, but no faster than the rate at which every ball along the optimal path keeps receiving samples forever."
      colorKey={['path', 'graph', 'goal']}
      caption={
        <>
          A path of length {L_PATH} in the unit square, tiled by {model.centers.length} balls of
          radius r_n/4 = {ballR.toFixed(4)}, with n = {n.toLocaleString()} uniform samples. Red balls
          received no sample. Expected empty balls: {model.centers.length} ·
          (1 − π r²)ⁿ = {expected.toFixed(2)}; observed {emptyCount}. Slide n: the balls shrink and
          multiply, but with γ at or above γ* their count times the emptiness probability goes to
          zero — Borel–Cantelli then says that, almost surely, every ball is hit for all large n, and
          consecutive balls within r_n of each other chain into a path in the random geometric graph
          that shadows the optimum. Pull γ/γ* below 1 and the expected count stops vanishing. For
          reference, k_n = ⌈e(1 + 1/d) log n⌉ = {kN(n, 2)} and the 2011 constant would give
          r_n = {rN(n, 2, gammaStarKf2011(2, 1) * mult).toFixed(4)}.
        </>
      }
    >
      <div className="flex justify-center p-3">
        <svg width={SIZE} height={SIZE} viewBox={`0 0 ${SIZE} ${SIZE}`} role="img" aria-label="A unit square with random samples, a curved path, and small balls tiling the path, some red where no sample landed inside." className="max-w-full">
          <rect x={PAD} y={PAD} width={SIZE - 2 * PAD} height={SIZE - 2 * PAD} fill="var(--color-fd-card)" stroke="var(--color-fd-border)" />
          {model.samples.length <= 4000
            ? model.samples.map((q, i) => <circle key={i} cx={px(q[0])} cy={py(q[1])} r={n > 1000 ? 0.9 : 1.6} fill="var(--rm-graph)" fillOpacity={0.55} />)
            : null}
          <polyline points={PATH.pts.map(([x, y]) => `${px(x)},${py(y)}`).join(' ')} fill="none" stroke="var(--rm-path)" strokeWidth={1.5} strokeOpacity={0.9} />
          {model.centers.map((c, i) => (
            <circle key={`b${i}`} cx={px(c[0])} cy={py(c[1])} r={Math.max(1, ballR * (SIZE - 2 * PAD))} fill={model.empty[i] ? 'var(--rm-goal)' : 'var(--rm-path)'} fillOpacity={model.empty[i] ? 0.6 : 0.15} stroke={model.empty[i] ? 'var(--rm-goal)' : 'var(--rm-path)'} strokeWidth={0.8} />
          ))}
          <text x={PAD + 4} y={PAD - 4} fontSize={10} fontFamily="ui-monospace, monospace" fill="currentColor">
            n = {n.toLocaleString()} · r_n = {r.toFixed(4)} · {model.centers.length} balls · {emptyCount} empty
          </text>
        </svg>
      </div>
      <ControlPanel columns={2}>
        <Slider label="samples n (log scale)" role="graph" value={logN} min={1.5} max={4.5} step={0.05} onChange={setLogN} format={() => n.toLocaleString()} />
        <Slider label="γ / γ*" role="path" value={mult} min={0.5} max={2} step={0.05} onChange={setMult} format={(x) => `${x.toFixed(2)}${x < 1 ? ' — below the bound' : ''}`} />
      </ControlPanel>
    </WidgetFrame>
  );
}
