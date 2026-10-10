import { CELL_BENCH } from '@/lib/cells/examples';
import { levelSegments, rasterCounts, rasterMorse, sliceField, SLICE_LABELS, type SliceSpec } from '@/lib/cells/slices';
import { worldBounds } from '@/lib/cells/world';
import type { Point2 } from '@/lib/sim/world';

/**
 * f10.4 — the slice-function gallery (Choset Figs. 6.12–6.18) on the §3 bench.
 *
 * One definition (Def. 6.2.1: cells are the components of Q_free \ I*), six
 * slice functions, six decompositions — each computed by the same raster
 * Morse decomposition (`rasterMorse`) and drawn as static SVG: cells tinted,
 * the critical slices I* dark, critical points as dots, and a handful of
 * level sets h = λ as the coverage pattern a robot would drive. Hover a panel
 * for what to notice.
 */

const CELL = 0.08;

interface PanelSpec {
  spec: SliceSpec;
  title: string;
  hint: string;
  levels: (hmin: number, hmax: number) => number[];
}

const range = (a: number, b: number, d: number) => {
  const out: number[] = [];
  for (let x = a; x <= b + 1e-9; x += d) out.push(x);
  return out;
};

const PANELS: PanelSpec[] = [
  { spec: { kind: 'vertical' }, title: 'boustrophedon', hint: 'Straight laps; cells change only where an obstacle starts or ends.', levels: (a, b) => range(a + 0.4, b, 0.8) },
  { spec: { kind: 'radial', center: { x: 5, y: 3 } }, title: 'spiral', hint: 'Circles about the centre; a circle that merely touches the outer wall opens into an arc without changing connectivity, so not every tangency is critical.', levels: (a, b) => range(a + 0.4, b, 0.8) },
  { spec: { kind: 'spike', center: { x: -3, y: -2.5 } }, title: 'spike', hint: 'Rays from a centre placed off the plot, below and to the left, where the angle is smooth (inside the plot its branch cut would act as one more wall); the cells are slices of a pie.', levels: (a, b) => range(a + 0.02, b, 0.06) },
  { spec: { kind: 'squarel', center: { x: 5, y: 3 } }, title: 'squarel', hint: 'Diamonds of straight laps — a spiral a differential-drive robot can drive.', levels: (a, b) => range(a + 0.4, b, 0.8) },
  { spec: { kind: 'brushfire' }, title: 'brushfire', hint: 'Fronts at fixed distance from every obstacle; cells are born at each obstacle and change where fronts collide. Each obstacle is a whole critical set, so the Reeb graph is a tree.', levels: (a, b) => range(a + 2, b, 6) },
  { spec: { kind: 'wavefront', center: { x: 1.5, y: 0.8 } }, title: 'wave-front', hint: 'Path length from a tether point; fronts wrap around obstacles and collide behind them.', levels: (a, b) => range(a + 2, b, 8) },
];

/** Join marching-squares segments that share endpoints into polylines, as one SVG path string. */
function chainSegments(segs: [Point2, Point2][], X: (x: number) => string, Y: (y: number) => string): string {
  const key = (q: Point2) => `${q.x.toFixed(5)},${q.y.toFixed(5)}`;
  const ends = new Map<string, number[]>();
  segs.forEach(([a, b], i) => {
    for (const q of [a, b]) {
      const k = key(q);
      if (!ends.has(k)) ends.set(k, []);
      ends.get(k)!.push(i);
    }
  });
  const used = new Uint8Array(segs.length);
  const out: string[] = [];
  for (let s0 = 0; s0 < segs.length; s0++) {
    if (used[s0]) continue;
    used[s0] = 1;
    const line: Point2[] = [segs[s0][0], segs[s0][1]];
    // Extend forward from the tail, then backward from the head.
    for (const forward of [true, false]) {
      for (;;) {
        const tip = forward ? line[line.length - 1] : line[0];
        const next = (ends.get(key(tip)) ?? []).find((i) => !used[i]);
        if (next === undefined) break;
        used[next] = 1;
        const [a, b] = segs[next];
        const other = key(a) === key(tip) ? b : a;
        if (forward) line.push(other);
        else line.unshift(other);
      }
    }
    out.push(`M${X(line[0].x)} ${Y(line[0].y)}` + line.slice(1).map((q) => `L${X(q.x)} ${Y(q.y)}`).join(''));
  }
  return out.join('');
}

function Panel({ p }: { p: PanelSpec }) {
  const m = rasterMorse(CELL_BENCH, p.spec, { cell: CELL });
  const { h } = sliceField(CELL_BENCH, m.raster, p.spec);
  let hmin = Infinity, hmax = -Infinity;
  for (let k = 0; k < h.length; k++) if (!Number.isNaN(h[k])) {
    hmin = Math.min(hmin, h[k]);
    hmax = Math.max(hmax, h[k]);
  }
  const segs = levelSegments(CELL_BENCH, m.raster, p.spec, p.levels(hmin, hmax), h);
  const c = rasterCounts(m);
  const b = worldBounds(CELL_BENCH);
  const pad = 0.2;
  const W = 280;
  const scale = W / (b.maxX - b.minX + 2 * pad);
  const H = (b.maxY - b.minY + 2 * pad) * scale;
  const X = (x: number) => ((x - b.minX + pad) * scale).toFixed(1);
  const Y = (y: number) => ((b.maxY + pad - y) * scale).toFixed(1);
  const poly = (pts: readonly Point2[]) => pts.map((q) => `${X(q.x)},${Y(q.y)}`).join(' ');
  const r = m.raster;
  // Run-length rows over 2 × 2 pixel blocks, grouped into one path per tint so
  // the static HTML stays small: "M x y h w v h h −w z" per run.
  const groups = new Map<string, { op: number; d: string[] }>();
  const B = 2;
  const ph = (B * r.cell * scale + 0.4).toFixed(1);
  const label = (bi: number, bj: number): string | null => {
    // A block is drawn as a critical slice if any of its pixels is one, else as its first cell.
    let cell = -1;
    for (let dj = 0; dj < B; dj++) for (let di = 0; di < B; di++) {
      const i = bi * B + di, j = bj * B + dj;
      if (i >= r.nx || j >= r.ny) continue;
      const k = j * r.nx + i;
      if (m.nodeOf[k] >= 0) return 'n';
      if (cell < 0 && m.cellOf[k] >= 0) cell = m.cellOf[k];
    }
    return cell < 0 ? null : `c${cell % 4}`;
  };
  const nbx = Math.ceil(r.nx / B), nby = Math.ceil(r.ny / B);
  for (let bj = 0; bj < nby; bj++) {
    let bi = 0;
    while (bi < nbx) {
      const key = label(bi, bj);
      if (key === null) {
        bi++;
        continue;
      }
      let e = bi + 1;
      while (e < nbx && label(e, bj) === key) e++;
      if (!groups.has(key)) groups.set(key, { op: key === 'n' ? 0.75 : 0.08 + 0.08 * Number(key.slice(1)), d: [] });
      const w = ((e - bi) * B * r.cell * scale + 0.4).toFixed(1);
      groups.get(key)!.d.push(`M${X(r.x0 + bi * B * r.cell)} ${Y(r.y0 + (bj + 1) * B * r.cell)}h${w}v${ph}h-${w}z`);
      bi = e;
    }
  }
  const pattern = chainSegments(segs, X, Y);
  const tree = p.spec.kind === 'brushfire';
  const euler = c.nCe === c.nCp + c.nOb - 1;
  const lab = SLICE_LABELS[p.spec.kind];
  return (
    <div className="group flex flex-col gap-1 rounded-sm p-1 transition-shadow hover:ring-1 hover:ring-fd-border" title={p.hint}>
      <svg viewBox={`0 0 ${W} ${H.toFixed(1)}`} width="100%" role="img" aria-label={`${p.title} decomposition of the bench`} className="max-w-[320px]">
        <rect x={0} y={0} width={W} height={H} fill="var(--pr-canvas-bg, transparent)" />
        {[...groups.entries()].map(([key, g]) => (
          <path key={key} d={g.d.join('')} fill="var(--rm-graph)" fillOpacity={g.op} />
        ))}
        <path d={pattern} fill="none" stroke="var(--rm-path)" strokeWidth={0.7} strokeOpacity={0.75} />
        <polygon points={poly(CELL_BENCH.boundary)} fill="none" stroke="var(--rm-obstacle)" strokeWidth={2} />
        {CELL_BENCH.obstacles.map((o, i) => (
          <polygon key={i} points={poly(o)} fill="var(--rm-obstacle)" fillOpacity={0.9} stroke="var(--rm-obstacle)" />
        ))}
        {m.nodes.map((n) => (
          <circle key={n.id} cx={X(n.q.x)} cy={Y(n.q.y)} r={3.2} fill="var(--rm-graph)" stroke="var(--pr-canvas-bg, transparent)" strokeWidth={1} />
        ))}
        {p.spec.center && p.spec.kind !== 'wavefront' ? <circle cx={X(p.spec.center.x)} cy={Y(p.spec.center.y)} r={2.6} fill="var(--pr-widget-ink)" /> : null}
        {p.spec.kind === 'wavefront' && p.spec.center ? <circle cx={X(p.spec.center.x)} cy={Y(p.spec.center.y)} r={3.2} fill="var(--rm-start)" /> : null}
      </svg>
      <span className="font-ui text-[0.72rem] leading-snug text-fd-muted-foreground">
        <span className="font-semibold text-fd-foreground">{p.title}</span> · {lab.formula} · N_cp {c.nCp}, N_ce {c.nCe}
        {tree ? ' · a tree: N_ce = N_cp − 1' : euler ? ` · ${c.nCp} + ${c.nOb} − 1 ✓` : ''}
      </span>
    </div>
  );
}

export function SliceGallery() {
  return (
    <div className="not-prose grid grid-cols-1 gap-3 sm:grid-cols-2 lg:grid-cols-3">
      {PANELS.map((p) => (
        <Panel key={p.title} p={p} />
      ))}
    </div>
  );
}
