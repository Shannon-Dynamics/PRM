'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { armRaster, componentReport, seamConnection, withJointLimits } from '@/lib/manifold/lab';
import {
  T2,
  chartDistanceT2,
  crossesSeam,
  foldSquare,
  geodesicPiecesOnChart,
  type T2Point,
  type Vec3,
} from '@/lib/manifold/torus';
import { TAU, wrapAngle } from '@/lib/manifold/s1';

/**
 * w5.1 — the Torus Unwrapper.
 *
 * Left: the [−π, π]² chart of Chapter 4 — amber C-obstacles, a purple geodesic
 * between two draggable endpoints, an orange configuration dot. Right: the same
 * square being folded into a cylinder and then a torus by `foldSquare`, carrying
 * obstacles and path along. A path that is two pieces on the square is one
 * curve on the torus; `T2.dist` is shorter than the chart's Euclidean distance
 * exactly when the short way crosses a seam. Joint limits cut the torus open
 * into a disc, after which no path may cross.
 */

const N = 40; // raster resolution
const MESH = 28; // fold mesh resolution per axis
const LIMIT = 3.0; // joint limit in radians, when enabled

interface SimState {
  fold: number;
}

/** Fold schedule: hold flat, fold up, hold doughnut, unfold. One cycle = 320 ticks. */
function foldAt(tick: number): number {
  const t = tick % 320;
  if (t < 40) return 0;
  if (t < 160) return (t - 40) / 120;
  if (t < 240) return 1;
  return 1 - (t - 240) / 80;
}

/** Orthographic camera: tilt about x then spin about z, then drop depth. */
function project(p: Vec3): [number, number, number] {
  const tilt = -1.05;
  const spin = 0.55;
  const cs = Math.cos(spin), ss = Math.sin(spin);
  const x1 = cs * p[0] - ss * p[1];
  const y1 = ss * p[0] + cs * p[1];
  const z1 = p[2];
  const ct = Math.cos(tilt), st = Math.sin(tilt);
  const y2 = ct * y1 - st * z1;
  const z2 = st * y1 + ct * z1;
  return [x1, y2, z2];
}

type Handle = 'a' | 'b' | 'q' | null;

export function TorusUnwrapper() {
  const [a, setA] = useState<T2Point>([-2.6, 0.4]);
  const [b, setB] = useState<T2Point>([2.7, -0.3]);
  const [q, setQ] = useState<T2Point>([1.0, -2.2]);
  const [naive, setNaive] = useState(false);
  const [limits, setLimits] = useState(false);
  const [manualFold, setManualFold] = useState<number | null>(null);
  const dragging = useRef<Handle>(null);

  const raster = useMemo(() => {
    const base = armRaster(N);
    return limits ? withJointLimits(base, LIMIT, LIMIT) : base;
  }, [limits]);
  const report = useMemo(() => componentReport(raster), [raster]);
  const seamLink = useMemo(() => seamConnection(raster, report), [raster, report]);

  const init = useCallback((): SimState => ({ fold: 0 }), []);
  const step = useCallback((s: SimState, tick: number): SimState => ({ ...s, fold: foldAt(tick + 1) }), []);
  const sim = useSimulation<SimState>({ init, step, fps: 30, initialSeed: 5 });
  const fold = manualFold ?? sim.state.fold;

  // With joint limits the space is an open rectangle: the geodesic is the chart's straight line.
  const pieces = useMemo<T2Point[][]>(() => {
    if (limits) {
      const pts: T2Point[] = [];
      for (let i = 0; i <= 64; i++) pts.push([a[0] + (i / 64) * (b[0] - a[0]), a[1] + (i / 64) * (b[1] - a[1])]);
      return [pts];
    }
    return geodesicPiecesOnChart(a, b, 96);
  }, [a, b, limits]);
  const dist = limits ? chartDistanceT2(a, b) : T2.dist(a, b);
  const chartDist = chartDistanceT2(a, b);
  const seam = crossesSeam(a, b);

  // Panel geometry in canvas world units: left square at x∈[0.05,0.95], right view at x∈[1.1,1.95].
  const WORLD = { minX: 0, minY: 0, maxX: 2, maxY: 1 };
  const L = { x0: 0.05, x1: 0.95, y0: 0.05, y1: 0.95 };
  const toChart = (wx: number, wy: number): T2Point | null => {
    if (wx < L.x0 || wx > L.x1 || wy < L.y0 || wy > L.y1) return null;
    return [-Math.PI + ((wx - L.x0) / (L.x1 - L.x0)) * TAU, -Math.PI + ((wy - L.y0) / (L.y1 - L.y0)) * TAU];
  };
  const fromChart = (p: T2Point): [number, number] => [
    L.x0 + ((wrapAngle(p[0]) + Math.PI) / TAU) * (L.x1 - L.x0),
    L.y0 + ((wrapAngle(p[1]) + Math.PI) / TAU) * (L.y1 - L.y0),
  ];

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const c = toChart(w[0], w[1]);
      if (phase === 'down') {
        if (!c) return;
        const cands: [Handle, T2Point][] = [['q', q], ['a', a], ['b', b]];
        let best: Handle = null;
        let bestD = 0.45;
        for (const [h, p] of cands) {
          const d = T2.dist(p, c);
          if (d < bestD) { bestD = d; best = h; }
        }
        dragging.current = best ?? 'q';
        if (!best) setQ(c);
      } else if (phase === 'move') {
        if (!dragging.current || !c) return;
        if (dragging.current === 'a') setA(c);
        else if (dragging.current === 'b') setB(c);
        else setQ(c);
      } else dragging.current = null;
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [a, b, q],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const X = (x: number) => sx(v, x);
      const Y = (y: number) => sy(v, y);

      /* ---------------- left: the chart ---------------- */
      const cw = (L.x1 - L.x0) / N;
      const pxw = Math.ceil(Math.abs(X(cw) - X(0))) + 1;
      ctx.save();
      ctx.fillStyle = p.free;
      ctx.fillRect(X(L.x0), Y(L.y1), X(L.x1) - X(L.x0), Y(L.y0) - Y(L.y1));
      ctx.fillStyle = p.cobstacle;
      ctx.globalAlpha = 0.85;
      for (let j = 0; j < N; j++) {
        for (let i = 0; i < N; i++) {
          if (!raster.occupied[j * N + i]) continue;
          ctx.fillRect(X(L.x0 + i * cw), Y(L.y0 + (j + 1) * cw), pxw, pxw);
        }
      }
      ctx.globalAlpha = 1;
      // Chart grid.
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let k = 1; k < 8; k++) {
        const t = L.x0 + (k / 8) * (L.x1 - L.x0);
        ctx.moveTo(X(t), Y(L.y0)); ctx.lineTo(X(t), Y(L.y1));
        ctx.moveTo(X(L.x0), Y(t)); ctx.lineTo(X(L.x1), Y(t));
      }
      ctx.stroke();
      // Seams: dashed teal, or solid when joint limits make them real boundaries.
      ctx.strokeStyle = p.accent;
      ctx.lineWidth = 2;
      ctx.setLineDash(limits ? [] : [6, 5]);
      ctx.strokeRect(X(L.x0), Y(L.y1), X(L.x1) - X(L.x0), Y(L.y0) - Y(L.y1));
      ctx.setLineDash([]);
      // Geodesic pieces.
      ctx.strokeStyle = p.path;
      ctx.lineWidth = 2.6;
      ctx.lineJoin = 'round';
      for (const piece of pieces) {
        ctx.beginPath();
        piece.forEach((pt, i) => {
          const [wx, wy] = fromChart(pt);
          if (i === 0) ctx.moveTo(X(wx), Y(wy)); else ctx.lineTo(X(wx), Y(wy));
        });
        ctx.stroke();
      }
      // The seam-crossing connection the lab found (only meaningful without limits).
      if (!limits && seamLink) {
        ctx.globalAlpha = 0.5;
        ctx.setLineDash([3, 3]);
        for (const piece of geodesicPiecesOnChart(seamLink.a, seamLink.b, 32)) {
          ctx.beginPath();
          piece.forEach((pt, i) => {
            const [wx, wy] = fromChart(pt);
            if (i === 0) ctx.moveTo(X(wx), Y(wy)); else ctx.lineTo(X(wx), Y(wy));
          });
          ctx.stroke();
        }
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
      }
      // Endpoints and the configuration dot.
      const dot = (pt: T2Point, color: string, r: number, hollow = false) => {
        const [wx, wy] = fromChart(pt);
        ctx.beginPath();
        ctx.arc(X(wx), Y(wy), r, 0, Math.PI * 2);
        if (hollow) { ctx.strokeStyle = color; ctx.lineWidth = 2.2; ctx.fillStyle = p.bg; ctx.fill(); ctx.stroke(); }
        else { ctx.fillStyle = color; ctx.fill(); }
      };
      dot(a, p.path, 6, true);
      dot(b, p.path, 6);
      dot(q, p.robot, 7);
      label(ctx, 'θ₁', X(L.x1) - 14, Y(L.y0) + 12, p.ink, { size: 11 });
      label(ctx, 'θ₂', X(L.x0) + 4, Y(L.y1) + 10, p.ink, { size: 11 });
      label(ctx, '−π', X(L.x0) - 2, Y(L.y0) + 12, p.ink, { size: 10 });
      label(ctx, 'π', X(L.x1) - 28, Y(L.y0) + 12, p.ink, { size: 10 });
      label(ctx, limits ? 'the chart, with joint limits: an open rectangle' : 'the chart [−π, π]² — edges identified', X(L.x0), Y(L.y1) - 10, p.ink, { size: 11, weight: 600 });
      ctx.restore();

      /* ---------------- right: the fold ---------------- */
      // Mesh vertices through the fold, projected.
      const verts: [number, number, number][] = new Array((MESH + 1) * (MESH + 1));
      for (let j = 0; j <= MESH; j++) {
        for (let i = 0; i <= MESH; i++) {
          const u = -Math.PI + (i / MESH) * TAU;
          const vv = -Math.PI + (j / MESH) * TAU;
          verts[j * (MESH + 1) + i] = project(foldSquare(u, vv, fold));
        }
      }
      // Fit the projection into the right panel.
      let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
      for (const [x, y] of verts) { minx = Math.min(minx, x); maxx = Math.max(maxx, x); miny = Math.min(miny, y); maxy = Math.max(maxy, y); }
      const R = { x0: 1.1, x1: 1.95, y0: 0.08, y1: 0.95 };
      const scale = Math.min((R.x1 - R.x0) / (maxx - minx + 1e-9), (R.y1 - R.y0) / (maxy - miny + 1e-9)) * 0.92;
      const cx = (R.x0 + R.x1) / 2, cy = (R.y0 + R.y1) / 2;
      const mx = (minx + maxx) / 2, my = (miny + maxy) / 2;
      const P = (pt: [number, number, number]): [number, number] => [X(cx + (pt[0] - mx) * scale), Y(cy + (pt[1] - my) * scale)];

      // Quads sorted back to front, shaded by their normal.
      type Quad = { i: number; j: number; depth: number; shade: number };
      const quads: Quad[] = [];
      for (let j = 0; j < MESH; j++) {
        for (let i = 0; i < MESH; i++) {
          const v00 = verts[j * (MESH + 1) + i], v10 = verts[j * (MESH + 1) + i + 1];
          const v01 = verts[(j + 1) * (MESH + 1) + i], v11 = verts[(j + 1) * (MESH + 1) + i + 1];
          const ex = [v10[0] - v00[0], v10[1] - v00[1], v10[2] - v00[2]];
          const ey = [v01[0] - v00[0], v01[1] - v00[1], v01[2] - v00[2]];
          const nz = ex[0] * ey[1] - ex[1] * ey[0];
          const nx = ex[1] * ey[2] - ex[2] * ey[1];
          const ny = ex[2] * ey[0] - ex[0] * ey[2];
          const nn = Math.hypot(nx, ny, nz) || 1;
          // Light from the upper-left-front.
          const lightDot = (nx * -0.3 + ny * 0.5 + nz * 0.81) / nn;
          quads.push({ i, j, depth: (v00[2] + v10[2] + v01[2] + v11[2]) / 4, shade: Math.abs(lightDot) });
        }
      }
      quads.sort((q1, q2) => q1.depth - q2.depth);
      ctx.save();
      for (const qd of quads) {
        const v00 = P(verts[qd.j * (MESH + 1) + qd.i]), v10 = P(verts[qd.j * (MESH + 1) + qd.i + 1]);
        const v01 = P(verts[(qd.j + 1) * (MESH + 1) + qd.i]), v11 = P(verts[(qd.j + 1) * (MESH + 1) + qd.i + 1]);
        // Occupancy of this mesh quad: sample the raster at its centre.
        const ri = Math.min(N - 1, Math.floor(((qd.i + 0.5) / MESH) * N));
        const rj = Math.min(N - 1, Math.floor(((qd.j + 0.5) / MESH) * N));
        const occ = raster.occupied[rj * N + ri] === 1;
        ctx.beginPath();
        ctx.moveTo(v00[0], v00[1]); ctx.lineTo(v10[0], v10[1]); ctx.lineTo(v11[0], v11[1]); ctx.lineTo(v01[0], v01[1]); ctx.closePath();
        ctx.fillStyle = occ ? p.cobstacle : p.unknown;
        ctx.globalAlpha = occ ? 0.95 : 0.8;
        ctx.fill();
        // Shade: darken by (1 − light).
        ctx.fillStyle = p.ink;
        ctx.globalAlpha = 0.45 * (1 - qd.shade);
        ctx.fill();
        ctx.globalAlpha = 0.35;
        ctx.strokeStyle = p.grid;
        ctx.lineWidth = 0.6;
        ctx.stroke();
      }
      ctx.globalAlpha = 1;
      // Seams on the surface: the edges u = ±π and v = ±π.
      ctx.strokeStyle = p.accent;
      ctx.lineWidth = 1.6;
      ctx.setLineDash(limits ? [] : [5, 4]);
      const edge = (pts: Vec3[]) => {
        ctx.beginPath();
        pts.forEach((pt, k) => { const [x, y] = P(project(pt)); if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
        ctx.stroke();
      };
      const seamPts = (fixedU: boolean, val: number) => {
        const out: Vec3[] = [];
        for (let k = 0; k <= MESH; k++) {
          const t = -Math.PI + (k / MESH) * TAU;
          out.push(fixedU ? foldSquare(val, t, fold) : foldSquare(t, val, fold));
        }
        return out;
      };
      edge(seamPts(true, -Math.PI)); edge(seamPts(true, Math.PI)); edge(seamPts(false, -Math.PI)); edge(seamPts(false, Math.PI));
      ctx.setLineDash([]);
      // Path and points on the surface.
      ctx.strokeStyle = p.path;
      ctx.lineWidth = 2.6;
      for (const piece of pieces) {
        ctx.beginPath();
        piece.forEach((pt, k) => {
          const [x, y] = P(project(foldSquare(pt[0], pt[1], fold)));
          if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
      }
      const dot3 = (pt: T2Point, color: string, r: number, hollow = false) => {
        const [x, y] = P(project(foldSquare(wrapAngle(pt[0]), wrapAngle(pt[1]), fold)));
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        if (hollow) { ctx.strokeStyle = color; ctx.lineWidth = 2.2; ctx.fillStyle = p.bg; ctx.fill(); ctx.stroke(); }
        else { ctx.fillStyle = color; ctx.fill(); }
      };
      dot3(a, p.path, 5, true);
      dot3(b, p.path, 5);
      dot3(q, p.robot, 6);
      const stage = fold < 0.02 ? 'the square' : fold < 0.5 ? 'folding θ₁ — the first identification' : fold < 0.98 ? 'folding θ₂ — the second identification' : 'the torus T²';
      label(ctx, stage, X(R.x0), Y(R.y1) - 10, p.ink, { size: 11, weight: 600 });
      ctx.restore();
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [a, b, q, fold, pieces, raster, limits, seamLink],
  );

  return (
    <WidgetFrame
      id="w5.1"
      title="Torus Unwrapper"
      teaches="T² is not a square with periodic boundary conditions. The square is one chart; the torus is the space, and the seam is where the chart lies."
      colorKey={['cobstacle', 'path', 'robot']}
      wide
      caption={
        <>
          Left, Reach&rsquo;s configuration space drawn the way Chapter 4 drew it: the chart
          [−π, π]² with amber C-obstacles. Right, the same square folded by the identification of
          its edges — first θ₁, then θ₂ — into a torus. The purple geodesic between the two
          handles is two pieces on the square and one curve on the doughnut; drag a handle across
          a dashed seam and watch <code>dist</code> stay small while the chart distance jumps.
          The orange dot is the configuration. Turn on joint limits: the seams become walls, the
          surface is a disc, and the component count on the square and the torus agree.
        </>
      }
    >
      <SimCanvas
        world={WORLD}
        draw={draw}
        deps={[fold, a, b, q, limits, naive]}
        aspect={2}
        padding={0}
        onPointer={onPointer}
        ariaLabel="A square chart of the torus with amber C-obstacles and a purple geodesic, next to a 3-D rendering of the same square folding into a torus with the obstacles and the path carried along."
      />
      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border text-center sm:grid-cols-4">
        <Stat label={naive ? 'chart |a − b| (naive)' : 'dist(a, b) on T²'} value={(naive ? chartDist : dist).toFixed(4)} />
        <Stat label={naive ? 'dist(a, b) on T²' : 'chart |a − b|'} value={(naive ? dist : chartDist).toFixed(4)} />
        <Stat label="seam crossed" value={limits ? 'forbidden' : seam[0] || seam[1] ? `yes (${seam[0] ? 'θ₁' : ''}${seam[0] && seam[1] ? ', ' : ''}${seam[1] ? 'θ₂' : ''})` : 'no'} />
        <Stat label="components square → torus" value={`${report.onSquare} → ${report.onTorus}`} />
      </div>
      <ControlPanel columns={3}>
        <Slider
          label="Fold"
          value={fold}
          min={0}
          max={1}
          step={0.005}
          onChange={(v) => { sim.pause(); setManualFold(v); }}
          format={(v) => (v < 0.02 ? 'square' : v < 0.5 ? `cylinder ${(v * 200).toFixed(0)}%` : v < 0.98 ? `torus ${((v - 0.5) * 200).toFixed(0)}%` : 'torus')}
          help="0 = the flat chart; ½ = a cylinder (θ₁ identified); 1 = the torus."
        />
        <Toggle label="Readout: naive chart distance first" checked={naive} onChange={setNaive} />
        <Toggle label={`Joint limits |θᵢ| < ${LIMIT}`} role="cobstacle" checked={limits} onChange={setLimits} />
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={() => { setManualFold(null); sim.toggle(); }}
        onStep={() => { setManualFold(null); sim.stepOnce(); }}
        onReset={() => { setManualFold(null); setA([-2.6, 0.4]); setB([2.7, -0.3]); setQ([1.0, -2.2]); sim.reset(); }}
        onReseed={() => { setManualFold(null); sim.reseed(); }}
        seed={sim.seed}
        tick={sim.tick}
      />
    </WidgetFrame>
  );
}

function Stat({ label: l, value }: { label: string; value: string }) {
  return (
    <div className="px-2 py-1.5">
      <div className="eyebrow">{l}</div>
      <div className="font-mono text-sm tabular-nums">{value}</div>
    </div>
  );
}
