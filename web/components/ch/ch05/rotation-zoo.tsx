'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import {
  axisAngle,
  eulerFromMatrix,
  quatFromAxisAngle,
  quatMul,
  quatToMatrix,
  rotateVec,
  so3Wrap,
  type EulerConvention,
  type Quat,
} from '@/lib/manifold/so3';
import type { Vec3 } from '@/lib/manifold/torus';
import { wrapAngle } from '@/lib/manifold/s1';

/**
 * w5.2 — the Rotation Zoo.
 *
 * One rigid body (a toy Hitch) tumbles along a recorded, perfectly smooth
 * motion. Three readouts track the same motion as time series: Z-Y-Z Euler
 * angles, axis–angle, and the unit quaternion. The motion is steered to pass
 * within `miss` radians of R₃₃ = 1 — the orientation where the Euler chart
 * ends. There φ and ψ swing wildly in opposite directions while their sum stays
 * smooth (Choset eq. E.2, live); the quaternion never notices. Gimbal lock is a
 * missing chart, not a mechanical defect.
 */

const SAMPLES = 400;

interface SimState {
  t: number;
}

/**
 * The recorded motion. Z-Y-Z: the body z-axis sweeps from the equator through
 * polar angle `miss` at t = ½ and back, while the body spins about z. Z-Y-X:
 * the body x-axis sweeps through pitch = π/2 − miss at t = ½ instead, which is
 * where roll–pitch–yaw loses its chart.
 */
function motion(t: number, miss: number, conv: EulerConvention): Quat {
  const spin = quatFromAxisAngle([0, 0, 1], 1.5 * t);
  if (conv === 'ZYZ') {
    const sweep = quatFromAxisAngle([1, 0, 0], Math.PI * (t - 0.5));
    const tilt = quatFromAxisAngle([0, 1, 0], miss);
    return so3Wrap(quatMul(spin, quatMul(sweep, tilt)));
  }
  const tilt = quatFromAxisAngle([1, 0, 0], miss);
  const sweep = quatFromAxisAngle([0, 1, 0], Math.PI * t);
  return so3Wrap(quatMul(spin, quatMul(tilt, sweep)));
}

interface Series {
  euler: [number, number, number][];
  eulerSum: number[];
  locked: boolean[];
  rotvec: Vec3[];
  quat: Quat[];
  lockMetric: number[];
}

function record(miss: number, conv: EulerConvention): Series {
  const s: Series = { euler: [], eulerSum: [], locked: [], rotvec: [], quat: [], lockMetric: [] };
  let prev: Quat | null = null;
  for (let k = 0; k <= SAMPLES; k++) {
    const t = k / SAMPLES;
    let q = motion(t, miss, conv);
    // Keep the quaternion curve continuous: choose the sign nearest the previous sample.
    if (prev && q.w * prev.w + q.x * prev.x + q.y * prev.y + q.z * prev.z < 0) q = { w: -q.w, x: -q.x, y: -q.y, z: -q.z };
    prev = q;
    const R = quatToMatrix(q);
    const e = eulerFromMatrix(conv, R, 1e-6);
    if (e.ok) {
      s.euler.push(e.angles);
      s.eulerSum.push(conv === 'ZYZ' ? wrapAngle(e.angles[0] + e.angles[2]) : wrapAngle(e.angles[0] - e.angles[2]));
      s.locked.push(false);
    } else {
      s.euler.push([NaN, conv === 'ZYZ' ? 0 : Math.PI / 2, NaN]);
      s.eulerSum.push(e.sum);
      s.locked.push(true);
    }
    const aa = axisAngle(q);
    s.rotvec.push([aa.axis[0] * aa.angle, aa.axis[1] * aa.angle, aa.axis[2] * aa.angle]);
    s.quat.push(q);
    s.lockMetric.push(conv === 'ZYZ' ? R[2][2] : -R[2][0]);
  }
  return s;
}

/** Hitch as a toy: a chassis box, a cab, and a wheel on each corner — enough to read its attitude. */
const BODY: { pts: Vec3[]; faces: number[][] } = (() => {
  const box = (cx: number, cy: number, cz: number, lx: number, ly: number, lz: number): Vec3[] => {
    const out: Vec3[] = [];
    for (const sxn of [-1, 1]) for (const syn of [-1, 1]) for (const szn of [-1, 1]) out.push([cx + sxn * lx / 2, cy + syn * ly / 2, cz + szn * lz / 2]);
    return out;
  };
  const faces = (o: number) => [
    [o + 0, o + 1, o + 3, o + 2], [o + 4, o + 5, o + 7, o + 6],
    [o + 0, o + 1, o + 5, o + 4], [o + 2, o + 3, o + 7, o + 6],
    [o + 0, o + 2, o + 6, o + 4], [o + 1, o + 3, o + 7, o + 5],
  ];
  const pts = [...box(0, 0, 0, 1.6, 0.8, 0.36), ...box(0.35, 0, 0.36, 0.6, 0.7, 0.36)];
  return { pts, faces: [...faces(0), ...faces(8)] };
})();

function projectIso(p: Vec3): [number, number, number] {
  const tilt = -1.0, spin = 0.7;
  const cs = Math.cos(spin), ss = Math.sin(spin);
  const x1 = cs * p[0] - ss * p[1], y1 = ss * p[0] + cs * p[1];
  const ct = Math.cos(tilt), st = Math.sin(tilt);
  return [x1, ct * y1 - st * p[2], st * y1 + ct * p[2]];
}

export function RotationZoo() {
  const [miss, setMiss] = useState(0.08);
  const [conv, setConv] = useState<EulerConvention>('ZYZ');
  const [manualT, setManualT] = useState<number | null>(null);

  const series = useMemo(() => record(miss, conv), [miss, conv]);

  const init = useCallback((): SimState => ({ t: 0 }), []);
  const step = useCallback((s: SimState, tick: number): SimState => ({ ...s, t: ((tick + 1) % 480) / 480 }), []);
  const sim = useSimulation<SimState>({ init, step, fps: 30, initialSeed: 52 });
  const t = manualT ?? sim.state.t;
  const k = Math.min(SAMPLES, Math.round(t * SAMPLES));

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const q = series.quat[k];

      /* ---- the body, left third ---- */
      const cx = sx(v, 0.33), cy = sy(v, 0.5);
      const scale = (sx(v, 0.6) - sx(v, 0)) / 3.2;
      const world = BODY.pts.map((pt) => projectIso(rotateVec(q, pt)));
      type Face = { idx: number[]; depth: number; shade: number };
      const faces: Face[] = BODY.faces.map((idx) => {
        const a = world[idx[0]], b = world[idx[1]], c = world[idx[2]];
        const ex = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], ey = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        const nz = ex[0] * ey[1] - ex[1] * ey[0], nx = ex[1] * ey[2] - ex[2] * ey[1], ny = ex[2] * ey[0] - ex[0] * ey[2];
        const nn = Math.hypot(nx, ny, nz) || 1;
        return { idx, depth: idx.reduce((s, i) => s + world[i][2], 0) / idx.length, shade: Math.abs((nx * -0.3 + ny * 0.4 + nz * 0.86) / nn) };
      });
      faces.sort((f1, f2) => f1.depth - f2.depth);
      ctx.save();
      for (const f of faces) {
        ctx.beginPath();
        f.idx.forEach((i, n) => { const [x, y] = world[i]; if (n === 0) ctx.moveTo(cx + x * scale, cy - y * scale); else ctx.lineTo(cx + x * scale, cy - y * scale); });
        ctx.closePath();
        ctx.fillStyle = p.robot;
        ctx.globalAlpha = 0.9;
        ctx.fill();
        ctx.fillStyle = p.ink;
        ctx.globalAlpha = 0.5 * (1 - f.shade);
        ctx.fill();
        ctx.globalAlpha = 1;
        ctx.strokeStyle = p.bg;
        ctx.lineWidth = 1;
        ctx.stroke();
      }
      // Body axes: x (graph), y (path), z (ink), and the world z as a reference.
      const axis = (dir: Vec3, color: string, name: string) => {
        const e = projectIso(rotateVec(q, dir));
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + e[0] * scale, cy - e[1] * scale);
        ctx.stroke();
        label(ctx, name, cx + e[0] * scale * 1.12, cy - e[1] * scale * 1.12, color, { size: 10, align: 'center', weight: 700 });
      };
      axis([1.3, 0, 0], p.graph, 'x');
      axis([0, 1.3, 0], p.path, 'y');
      axis([0, 0, 1.3], p.ink, 'z');
      const wz = projectIso([0, 0, 1.45]);
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = p.accent;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + wz[0] * scale, cy - wz[1] * scale);
      ctx.stroke();
      ctx.setLineDash([]);
      label(ctx, conv === 'ZYZ' ? 'world z' : 'world z (pitch lock when body x is vertical)', cx + wz[0] * scale, cy - wz[1] * scale - 10, p.accent, { size: 9, align: 'center' });
      const lockVal = series.lockMetric[k];
      label(ctx, conv === 'ZYZ' ? `R₃₃ = ${lockVal.toFixed(3)}` : `−R₃₁ = sin θ = ${lockVal.toFixed(3)}`, sx(v, 0.33), sy(v, 0.06), series.locked[k] ? p.goal : p.ink, { size: 11, align: 'center', weight: 600 });
      label(ctx, series.locked[k] ? 'GIMBAL LOCK — chart lost' : Math.abs(Math.abs(lockVal) - 1) < 0.02 ? 'approaching the lost chart' : 'Euler chart valid', sx(v, 0.33), sy(v, 0.02), series.locked[k] ? p.goal : p.ink, { size: 10, align: 'center' });
      ctx.restore();

      /* ---- three strip charts, right two-thirds ---- */
      const x0 = sx(v, 0.68), x1 = sx(v, 1.97);
      const strips: { title: string; rows: { name: string; color: string; data: number[]; dashed?: boolean }[]; lo: number; hi: number }[] = [
        {
          title: conv === 'ZYZ' ? 'Z-Y-Z Euler angles (φ, θ, ψ) and φ + ψ' : 'Z-Y-X roll–pitch–yaw (φ, θ, ψ) and φ − ψ',
          rows: [
            { name: 'φ', color: p.graph, data: series.euler.map((e) => e[0]) },
            { name: 'θ', color: p.robot, data: series.euler.map((e) => e[1]) },
            { name: 'ψ', color: p.path, data: series.euler.map((e) => e[2]) },
            { name: conv === 'ZYZ' ? 'φ+ψ' : 'φ−ψ', color: p.ink, data: series.eulerSum, dashed: true },
          ],
          lo: -Math.PI, hi: Math.PI,
        },
        {
          title: 'axis–angle θ·n (exponential coordinates)',
          rows: [
            { name: 'θnₓ', color: p.graph, data: series.rotvec.map((r) => r[0]) },
            { name: 'θn_y', color: p.robot, data: series.rotvec.map((r) => r[1]) },
            { name: 'θn_z', color: p.path, data: series.rotvec.map((r) => r[2]) },
          ],
          lo: -Math.PI, hi: Math.PI,
        },
        {
          title: 'unit quaternion (q₀, q₁, q₂, q₃)',
          rows: [
            { name: 'q₀', color: p.ink, data: series.quat.map((qq) => qq.w) },
            { name: 'q₁', color: p.graph, data: series.quat.map((qq) => qq.x) },
            { name: 'q₂', color: p.robot, data: series.quat.map((qq) => qq.y) },
            { name: 'q₃', color: p.path, data: series.quat.map((qq) => qq.z) },
          ],
          lo: -1, hi: 1,
        },
      ];
      const stripH = (sy(v, 0.04) - sy(v, 0.96)) / 3;
      strips.forEach((st, si) => {
        const top = sy(v, 0.96) + si * stripH + 14;
        const bottom = top + stripH - 26;
        const Yv = (val: number) => bottom - ((val - st.lo) / (st.hi - st.lo)) * (bottom - top);
        ctx.save();
        ctx.strokeStyle = p.grid;
        ctx.lineWidth = 1;
        ctx.strokeRect(x0, top, x1 - x0, bottom - top);
        ctx.beginPath();
        ctx.moveTo(x0, Yv(0)); ctx.lineTo(x1, Yv(0));
        ctx.stroke();
        label(ctx, st.title, x0, top - 8, p.ink, { size: 10, weight: 600 });
        st.rows.forEach((row, ri) => {
          ctx.strokeStyle = row.color;
          ctx.lineWidth = row.dashed ? 1.4 : 1.6;
          ctx.setLineDash(row.dashed ? [4, 3] : []);
          ctx.beginPath();
          let pen = false;
          for (let i = 0; i <= SAMPLES; i++) {
            const val = row.data[i];
            const x = x0 + (i / SAMPLES) * (x1 - x0);
            if (!Number.isFinite(val)) { pen = false; continue; }
            // Break the line across a 2π wrap rather than drawing a vertical slash.
            if (pen && i > 0 && Math.abs(val - row.data[i - 1]) > (st.hi - st.lo) * 0.5) pen = false;
            if (!pen) { ctx.moveTo(x, Yv(val)); pen = true; } else ctx.lineTo(x, Yv(val));
          }
          ctx.stroke();
          ctx.setLineDash([]);
          label(ctx, row.name, x1 + 6, top + 8 + ri * 12, row.color, { size: 9, weight: 700 });
        });
        // Lock region shading for the Euler strip.
        if (si === 0) {
          ctx.fillStyle = p.goal;
          ctx.globalAlpha = 0.12;
          for (let i = 0; i <= SAMPLES; i++) if (Math.abs(Math.abs(series.lockMetric[i]) - 1) < 0.02) ctx.fillRect(x0 + (i / SAMPLES) * (x1 - x0), top, (x1 - x0) / SAMPLES + 1, bottom - top);
          ctx.globalAlpha = 1;
        }
        // Time cursor.
        const xc = x0 + t * (x1 - x0);
        ctx.strokeStyle = p.accent;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(xc, top); ctx.lineTo(xc, bottom);
        ctx.stroke();
        ctx.restore();
      });
    },
    [series, k, t, conv],
  );

  const e = series.euler[k];
  const qq = series.quat[k];
  const rv = series.rotvec[k];

  return (
    <WidgetFrame
      id="w5.2"
      title="Rotation Zoo"
      teaches="Gimbal lock is not mechanical. It is a missing chart: Euler angles are coordinates on SO(3) minus a set, and the body simply drove off the map."
      colorKey={['robot']}
      wide
      caption={
        <>
          Hitch tumbles along one smooth motion; three readouts describe it. As the body&rsquo;s
          z-axis passes within <em>miss</em> radians of the world z-axis — R₃₃ → 1, the shaded
          band — the Z-Y-Z angles φ and ψ swing through nearly a half-turn in opposite directions
          while their dashed sum φ + ψ stays smooth (eq. E.2: at θ = 0 only the sum is defined).
          The axis–angle and quaternion curves do not notice; the quaternion&rsquo;s four charts
          cover SO(3). Set <em>miss</em> to zero to hit the lost chart exactly, or switch to
          roll–pitch–yaw and watch the same thing happen at pitch = π/2 instead.
        </>
      }
    >
      <SimCanvas
        world={{ minX: 0, minY: 0, maxX: 2, maxY: 1 }}
        draw={draw}
        deps={[k, series, conv]}
        aspect={2}
        padding={0}
        ariaLabel="A rotating box with body axes beside three time-series strips showing its Euler angles, axis-angle vector and unit quaternion, with a cursor at the current time; the Euler angles swing violently near gimbal lock while the others stay smooth."
      />
      <div className="grid grid-cols-3 divide-x divide-fd-border border-t border-fd-border text-center">
        <Stat label={`Euler ${conv} (φ, θ, ψ)`} value={series.locked[k] ? `lock: ${conv === 'ZYZ' ? 'φ+ψ' : 'φ−ψ'} = ${series.eulerSum[k].toFixed(3)}` : `${e[0].toFixed(2)}, ${e[1].toFixed(2)}, ${e[2].toFixed(2)}`} />
        <Stat label="axis–angle θ·n" value={`${rv[0].toFixed(2)}, ${rv[1].toFixed(2)}, ${rv[2].toFixed(2)}`} />
        <Stat label="quaternion" value={`${qq.w.toFixed(2)}, ${qq.x.toFixed(2)}, ${qq.y.toFixed(2)}, ${qq.z.toFixed(2)}`} />
      </div>
      <ControlPanel columns={3}>
        <Slider
          label="Scrub t"
          value={t}
          min={0}
          max={1}
          step={0.0025}
          onChange={(val) => { sim.pause(); setManualT(val); }}
          format={(val) => val.toFixed(3)}
        />
        <Slider
          label="Miss distance from the lost chart"
          value={miss}
          min={0}
          max={0.6}
          step={0.005}
          unit="rad"
          onChange={setMiss}
          help="How close the motion passes to R₃₃ = 1 (Z-Y-Z) or pitch = π/2 (Z-Y-X). Zero hits it."
        />
        <ButtonRow>
          <ActionButton emphasis={conv === 'ZYZ'} onClick={() => setConv('ZYZ')}>Z-Y-Z Euler</ActionButton>
          <ActionButton emphasis={conv === 'ZYX'} onClick={() => setConv('ZYX')}>Z-Y-X roll–pitch–yaw</ActionButton>
        </ButtonRow>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={() => { setManualT(null); sim.toggle(); }}
        onStep={() => { setManualT(null); sim.stepOnce(); }}
        onReset={() => { setManualT(null); sim.reset(); }}
        onReseed={() => { setManualT(null); setMiss(0.08); sim.reseed(); }}
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
      <div className="font-mono text-xs tabular-nums sm:text-sm">{value}</div>
    </div>
  );
}
