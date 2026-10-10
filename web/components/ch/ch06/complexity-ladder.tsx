/**
 * f6.4 — the complexity-class ladder, a static SVG.
 *
 * P ⊆ NP ⊆ PSPACE ⊆ EXPTIME as nested boxes (Choset App. G.2, Definitions
 * G.2.5–G.2.12), and the three completeness notions this book uses as law
 * (Definitions G.3.1–G.3.3). Hover any term for its definition. No client
 * JavaScript: `<title>` elements are the tooltips.
 */

const CLASSES = [
  {
    name: 'EXPTIME',
    def: 'Def. G.2.6 — decided by an algorithm whose running time is O(2^(n^c)) for some constant c.',
    w: 560,
    h: 150,
  },
  {
    name: 'PSPACE',
    def: 'Def. G.2.11 — decided by an algorithm using polynomial space. Generalized motion planning for many bodies is PSPACE-hard (Canny), which is why Chapter 9 can say the roadmap bound is tight.',
    w: 430,
    h: 112,
  },
  {
    name: 'NP',
    def: 'Def. G.2.7 — a candidate solution (certificate) of polynomial length can be verified in polynomial time. Whether P = NP is open.',
    w: 300,
    h: 76,
  },
  {
    name: 'P',
    def: 'Def. G.2.5 — decided in polynomial time: sorting, shortest paths, A* on an explicit graph.',
    w: 170,
    h: 40,
  },
];

const COMPLETENESS = [
  {
    name: 'complete',
    def: 'Def. G.3.1 — in finite time, finds a solution if one exists, otherwise reports that none does. A* on a finite graph.',
  },
  {
    name: 'resolution complete',
    def: 'Def. G.3.2 — the same guarantee, for a fixed resolution step ε > 0: a grid planner is complete relative to its grid.',
  },
  {
    name: 'probabilistically complete',
    def: 'Def. G.3.3 — the probability of finding a solution, if one exists, goes to 1 as running time goes to infinity. PRM and RRT, Part III.',
  },
];

export function ComplexityLadder() {
  const W = 760;
  const H = 230;
  const cx = 300;
  const baseY = 200;
  return (
    <svg
      viewBox={`0 0 ${W} ${H}`}
      role="img"
      aria-label="Nested boxes labelled P inside NP inside PSPACE inside EXPTIME, beside three stacked labels: complete, resolution complete, probabilistically complete."
      className="w-full"
      style={{ fontFamily: 'ui-monospace, monospace', fontSize: 12 }}
    >
      {CLASSES.map((c, i) => (
        <g key={c.name}>
          <title>{`${c.name}: ${c.def}`}</title>
          <rect
            x={cx - c.w / 2}
            y={baseY - c.h}
            width={c.w}
            height={c.h}
            rx={4}
            fill="var(--rm-graph)"
            fillOpacity={0.07 + i * 0.07}
            stroke="var(--rm-graph)"
            strokeOpacity={0.6}
            strokeWidth={1}
          />
          <text x={cx + c.w / 2 - 8} y={baseY - c.h + 16} textAnchor="end" fill="var(--color-fd-foreground)" fontWeight={600}>
            {c.name}
          </text>
        </g>
      ))}
      <text x={cx} y={baseY + 20} textAnchor="middle" fill="var(--color-fd-muted-foreground)">
        P ⊆ NP ⊆ PSPACE ⊆ EXPTIME (Choset §G.2) — inclusions proven, strictness open except P ⊊ EXPTIME
      </text>
      {COMPLETENESS.map((c, i) => (
        <g key={c.name}>
          <title>{`${c.name}: ${c.def}`}</title>
          <rect x={600} y={60 + i * 44} width={150} height={34} rx={4} fill="var(--rm-path)" fillOpacity={0.1 + i * 0.1} stroke="var(--rm-path)" strokeOpacity={0.6} />
          <text x={675} y={82 + i * 44} textAnchor="middle" fill="var(--color-fd-foreground)" fontWeight={600} fontSize={11}>
            {c.name}
          </text>
        </g>
      ))}
      <text x={675} y={48} textAnchor="middle" fill="var(--color-fd-muted-foreground)">
        three completeness notions (§G.3)
      </text>
      <text x={675} y={210} textAnchor="middle" fill="var(--color-fd-muted-foreground)" fontSize={10}>
        weaker as you go down; hover for definitions
      </text>
    </svg>
  );
}
