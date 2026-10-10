import { transversalityMargin, type Vec } from '@/lib/roadmap/gvg';

/**
 * f9.4 — the transversality trio (Choset Fig. 5.17): two lines in the plane
 * meet in one of three ways — not at all (parallel), at a point, or along a
 * line (overlap). Hover a panel to perturb the second line a little: the
 * parallel and overlapping pairs change the dimension of their intersection,
 * the crossing pair keeps a point. Under each panel, the smallest singular
 * value of DG — rows the two lines' normals, the quantity `transversalityMargin`
 * computes for the GVG's sheets — is zero exactly in the two unstable cases.
 * Static SVG; the hover is CSS only.
 */

const W = 220;
const H = 170;
const S = 26; // px per unit
const OX = W / 2;
const OY = H / 2 + 6;

type Line = { p: Vec; d: Vec };

const px = (q: Vec) => [OX + q[0] * S, OY - q[1] * S];

function segment(l: Line, t = 4.2): string {
  const [x1, y1] = px([l.p[0] - t * l.d[0], l.p[1] - t * l.d[1]]);
  const [x2, y2] = px([l.p[0] + t * l.d[0], l.p[1] + t * l.d[1]]);
  return `M${x1.toFixed(1)},${y1.toFixed(1)} L${x2.toFixed(1)},${y2.toFixed(1)}`;
}

const unit = (v: Vec): Vec => {
  const n = Math.hypot(v[0], v[1]);
  return [v[0] / n, v[1] / n];
};
const rot = (v: Vec, a: number): Vec => [Math.cos(a) * v[0] - Math.sin(a) * v[1], Math.sin(a) * v[0] + Math.cos(a) * v[1]];
const normal = (l: Line): Vec => [-l.d[1], l.d[0]];

/** Where two lines cross, or null when they are parallel. */
function crossing(a: Line, b: Line): Vec | null {
  const det = a.d[0] * -b.d[1] - a.d[1] * -b.d[0];
  if (Math.abs(det) < 1e-12) return null;
  const rx = b.p[0] - a.p[0];
  const ry = b.p[1] - a.p[1];
  const t = (rx * -b.d[1] - ry * -b.d[0]) / det;
  return [a.p[0] + t * a.d[0], a.p[1] + t * a.d[1]];
}

interface Case {
  title: string;
  formula: string;
  M1: Line;
  M2: Line;
  /** The perturbed second line shown on hover. */
  M2p: Line;
  verdict: string;
  after: string;
  stable: boolean;
}

const M1: Line = { p: [0, 0], d: unit([1, 0.35]) };
const tilt = 0.12;

const CASES: Case[] = [
  {
    title: 'parallel',
    formula: 'M₁ ∩ M₂ = ∅',
    M1,
    M2: { p: [0, 0.8], d: M1.d },
    M2p: { p: [0, 0.8], d: rot(M1.d, -2.5 * tilt) },
    verdict: 'unstable',
    after: 'perturbed: they meet at a point — the intersection changed dimension',
    stable: false,
  },
  {
    title: 'a point',
    formula: 'T_xM₁ + T_xM₂ = R²',
    M1,
    M2: { p: [0, 0], d: unit([-0.55, 1]) },
    M2p: { p: [0.25, 0.1], d: rot(unit([-0.55, 1]), tilt) },
    verdict: 'transversal — stable',
    after: 'perturbed: still a point, nearby — codimensions add, 1 + 1 = 2',
    stable: true,
  },
  {
    title: 'overlap',
    formula: 'T_xM₁ + T_xM₂ = T_xM₁ ≠ R²',
    M1,
    M2: { p: [0, 0], d: M1.d },
    M2p: { p: [0, 0.2], d: rot(M1.d, tilt) },
    verdict: 'unstable',
    after: 'perturbed: a point (or nothing) — the line of intersection is gone',
    stable: false,
  },
];

function Panel({ c }: { c: Case }) {
  const margin = transversalityMargin([normal(c.M1), normal(c.M2)]);
  const x = c.stable ? crossing(c.M1, c.M2) : null;
  const xp = crossing(c.M1, c.M2p);
  return (
    <div className="group flex flex-col items-center gap-1">
      <svg viewBox={`0 0 ${W} ${H}`} width="100%" role="img" aria-label={`Two lines in the plane, ${c.title}: ${c.verdict}`} className="max-w-[260px]">
        <rect x={0} y={0} width={W} height={H} fill="var(--pr-canvas-bg, transparent)" />
        <path d={segment(c.M1)} stroke="var(--rm-graph)" strokeWidth={c.title === 'overlap' ? 6 : 2.4} strokeOpacity={c.title === 'overlap' ? 0.45 : 1} fill="none" strokeLinecap="round" />
        <path d={segment(c.M2)} stroke="var(--pr-widget-ink)" strokeWidth={2.4} fill="none" strokeLinecap="round" className="transition-opacity duration-300 group-hover:opacity-20" />
        {x ? <circle cx={px(x)[0]} cy={px(x)[1]} r={4.5} fill="var(--rm-graph)" className="transition-opacity duration-300 group-hover:opacity-20" /> : null}
        <g className="opacity-0 transition-opacity duration-300 group-hover:opacity-100">
          <path d={segment(c.M2p)} stroke="var(--pr-widget-ink)" strokeWidth={2.2} strokeDasharray="5 4" fill="none" strokeLinecap="round" />
          {xp && Math.abs(xp[0]) < 4 && Math.abs(xp[1]) < 3.2 ? <circle cx={px(xp)[0]} cy={px(xp)[1]} r={4.5} fill="var(--rm-graph)" /> : null}
        </g>
        <text x={8} y={16} fontSize={11} fontWeight={600} fill="currentColor" className="font-ui">
          {c.title}
        </text>
        <text x={8} y={30} fontSize={10} fill="currentColor" opacity={0.75} className="font-mono">
          {c.formula}
        </text>
        <text x={W - 8} y={H - 8} fontSize={9.5} textAnchor="end" fill="currentColor" opacity={0.75} className="font-mono">
          σ_min(DG) = {margin.toFixed(3)}
        </text>
      </svg>
      <span className="font-ui text-[0.72rem] font-semibold" style={{ color: c.stable ? 'var(--rm-graph)' : 'var(--rm-goal)' }}>
        {c.verdict}
      </span>
      <span className="max-w-[240px] text-center font-ui text-[0.7rem] leading-snug text-fd-muted-foreground">{c.after}</span>
    </div>
  );
}

export function TransversalityTrio() {
  return (
    <div className="not-prose grid grid-cols-1 gap-4 sm:grid-cols-3" aria-label="Three ways two lines in the plane can intersect">
      {CASES.map((c) => (
        <Panel key={c.title} c={c} />
      ))}
    </div>
  );
}
