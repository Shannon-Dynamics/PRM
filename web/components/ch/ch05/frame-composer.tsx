'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { compose, inverse, inverseTransformPoint, transformPoint, type Pose2 } from '@/lib/geom/se2';
import { CHOSET_FRAMES, frameWorked, tidy, toMatrix } from '@/lib/manifold/se2';
import { wrapAngle } from '@/lib/manifold/s1';

/**
 * w5.4 — the Frame Composer.
 *
 * Choset's frames A, B, C and the point w on a unit grid (figure 3.17). Drag
 * frame B (its origin, or the tip of its x-axis to rotate it) and frame C, and
 * drag w; the readout recomputes T_AB T_BC = T_AC, w_B = T_BC w_C, w_A = T_AB w_B
 * live, with the cancelled subscripts dimmed. The body/world toggle applies T₁
 * on the right (B′ = T_AB T₁, body frame) and on the left (B″ = T₁ T_AB, world
 * frame) and draws both: a transform is a group element, and the order of
 * multiplication is the whole content.
 */

type Handle = 'B' | 'Brot' | 'C' | 'Crot' | 'w' | null;

interface SimState {
  /** T₁'s heading, swept by autoplay so B′ and B″ visibly diverge. */
  theta1: number;
}

const WORLD = { minX: -7.5, minY: -4.5, maxX: 4.5, maxY: 4.5 };

export function FrameComposer() {
  const [T_AB, setTAB] = useState<Pose2>(CHOSET_FRAMES.T_AB);
  const [T_BC, setTBC] = useState<Pose2>(CHOSET_FRAMES.T_BC);
  const [w_C, setWC] = useState<[number, number]>(CHOSET_FRAMES.w_C);
  const [showT1, setShowT1] = useState(true);
  const [showMatrices, setShowMatrices] = useState(false);
  const [t1y, setT1y] = useState(CHOSET_FRAMES.T1.y);
  const [manualTheta1, setManualTheta1] = useState<number | null>(null);
  const dragging = useRef<Handle>(null);

  const init = useCallback((): SimState => ({ theta1: Math.PI }), []);
  const step = useCallback((s: SimState, tick: number): SimState => ({ theta1: wrapAngle(Math.PI + 0.012 * (tick + 1)) }), []);
  const sim = useSimulation<SimState>({ init, step, fps: 30, initialSeed: 54 });
  const theta1 = manualTheta1 ?? sim.state.theta1;
  const T1: Pose2 = useMemo(() => ({ x: 0, y: t1y, theta: theta1 }), [t1y, theta1]);

  const worked = useMemo(() => frameWorked({ T_AB, T_BC, w_C, T1 }), [T_AB, T_BC, w_C, T1]);
  const T_AC = worked.T_AC;

  const onPointer = useCallback(
    (wpt: [number, number], phase: 'down' | 'move' | 'up') => {
      const [x, y] = wpt;
      if (phase === 'down') {
        const tipB = transformPoint(T_AB, [1, 0]);
        const tipC = transformPoint(T_AC, [1, 0]);
        const cands: [Handle, [number, number]][] = [
          ['w', worked.w_A],
          ['Brot', tipB],
          ['Crot', tipC],
          ['B', [T_AB.x, T_AB.y]],
          ['C', [T_AC.x, T_AC.y]],
        ];
        for (const [h, pt] of cands) {
          if (Math.hypot(pt[0] - x, pt[1] - y) < 0.45) { dragging.current = h; return; }
        }
        dragging.current = null;
      } else if (phase === 'move') {
        const h = dragging.current;
        if (!h) return;
        const snap = (val: number) => Math.round(val * 2) / 2;
        if (h === 'B') setTAB({ ...T_AB, x: snap(x), y: snap(y) });
        else if (h === 'Brot') setTAB({ ...T_AB, theta: Math.round(Math.atan2(y - T_AB.y, x - T_AB.x) / (Math.PI / 4)) * (Math.PI / 4) });
        else if (h === 'C') {
          const newAC: Pose2 = { ...T_AC, x: snap(x), y: snap(y) };
          setTBC(compose(inverse(T_AB), newAC));
        } else if (h === 'Crot') {
          const newAC: Pose2 = { ...T_AC, theta: Math.round(Math.atan2(y - T_AC.y, x - T_AC.x) / (Math.PI / 4)) * (Math.PI / 4) };
          setTBC(compose(inverse(T_AB), newAC));
        } else if (h === 'w') {
          const local = inverseTransformPoint(T_AC, [snap(x), snap(y)]);
          setWC([Math.round(local[0] * 2) / 2, Math.round(local[1] * 2) / 2]);
        }
      } else dragging.current = null;
    },
    [T_AB, T_AC, worked.w_A],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      const frame = (T: Pose2, color: string, name: string, opts: { dashed?: boolean; alpha?: number } = {}) => {
        const o: [number, number] = [T.x, T.y];
        const ex = transformPoint(T, [1, 0]);
        const ey = transformPoint(T, [0, 1]);
        ctx.save();
        ctx.globalAlpha = opts.alpha ?? 1;
        ctx.setLineDash(opts.dashed ? [5, 4] : []);
        ctx.lineWidth = 2.4;
        ctx.strokeStyle = color;
        ctx.beginPath();
        ctx.moveTo(sx(v, o[0]), sy(v, o[1])); ctx.lineTo(sx(v, ex[0]), sy(v, ex[1]));
        ctx.moveTo(sx(v, o[0]), sy(v, o[1])); ctx.lineTo(sx(v, ey[0]), sy(v, ey[1]));
        ctx.stroke();
        // Arrowhead on x so the orientation is unambiguous.
        const ang = Math.atan2(sy(v, ex[1]) - sy(v, o[1]), sx(v, ex[0]) - sx(v, o[0]));
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(sx(v, ex[0]), sy(v, ex[1]));
        ctx.lineTo(sx(v, ex[0]) - 9 * Math.cos(ang - 0.4), sy(v, ex[1]) - 9 * Math.sin(ang - 0.4));
        ctx.lineTo(sx(v, ex[0]) - 9 * Math.cos(ang + 0.4), sy(v, ex[1]) - 9 * Math.sin(ang + 0.4));
        ctx.closePath();
        ctx.fill();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.arc(sx(v, o[0]), sy(v, o[1]), 4, 0, Math.PI * 2);
        ctx.fill();
        label(ctx, `x̃_${name}`, sx(v, ex[0]) + 8, sy(v, ex[1]) - 8, color, { size: 10, weight: 700 });
        label(ctx, `ỹ_${name}`, sx(v, ey[0]) + 8, sy(v, ey[1]) - 8, color, { size: 10, weight: 700 });
        label(ctx, name, sx(v, o[0]) - 10, sy(v, o[1]) + 12, color, { size: 12, weight: 700, align: 'right' });
        ctx.restore();
      };
      frame({ x: 0, y: 0, theta: 0 }, p.ink, 'A');
      frame(T_AB, p.robot, 'B');
      frame(T_AC, p.graph, 'C');
      if (showT1) {
        frame(worked.body, p.robot, 'B′ = T_AB T₁', { dashed: true, alpha: 0.75 });
        frame(worked.world, p.path, 'B″ = T₁ T_AB', { dashed: true, alpha: 0.75 });
      }
      // The point w, and its displaced image T_AB w_A (figure 3.18).
      const wA = worked.w_A;
      ctx.save();
      ctx.fillStyle = p.accent;
      ctx.beginPath();
      ctx.arc(sx(v, wA[0]), sy(v, wA[1]), 6, 0, Math.PI * 2);
      ctx.fill();
      label(ctx, 'w', sx(v, wA[0]) + 9, sy(v, wA[1]) - 9, p.accent, { size: 12, weight: 700 });
      const wd = worked.w_displaced;
      ctx.globalAlpha = 0.6;
      ctx.strokeStyle = p.accent;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.arc(sx(v, wd[0]), sy(v, wd[1]), 6, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
      label(ctx, "w′ = T_AB w_A", sx(v, wd[0]) + 9, sy(v, wd[1]) + 10, p.accent, { size: 9.5 });
      ctx.restore();
      // Grid labels.
      ctx.save();
      for (let gx = Math.ceil(v.minX); gx <= Math.floor(v.maxX); gx++) if (gx % 2 === 0) label(ctx, String(gx), sx(v, gx), v.height - 8, p.truth, { size: 9, align: 'center' });
      ctx.restore();
    },
    [T_AB, T_AC, worked, showT1],
  );

  const pose = (T: Pose2) => `(${fmt(T.x)}, ${fmt(T.y)}, ${fmtAngle(T.theta)})`;
  const pt = (q: [number, number]) => `(${fmt(q[0])}, ${fmt(q[1])})`;

  return (
    <WidgetFrame
      id="w5.4"
      title="Frame Composer"
      teaches="A transform is not a position plus an angle. It is a group element, and the order of multiplication is the whole content: T_AB T₁ and T₁ T_AB place B differently."
      colorKey={['robot', 'graph', 'path']}
      wide
      caption={
        <>
          Choset&rsquo;s three frames on a unit grid (figure 3.17): A fixed, B (orange) and C (blue)
          draggable by their origins and rotatable by the tips of their x-axes, the point w (teal)
          draggable too. The readout recomputes T_AB T_BC = T_AC by subscript cancellation and
          carries w from C to B to A. The dashed frames are figure 3.19: B′ is B displaced by T₁
          <em>in its own frame</em> (right multiplication), B″ is B displaced by the same T₁{' '}
          <em>in A&rsquo;s frame</em> (left multiplication). Autoplay sweeps T₁&rsquo;s heading;
          reset returns to Choset&rsquo;s numbers, where T₁ = (0, 1, π) gives B′ = (−2, −1, 0)
          and B″ = (2, 1, 0).
        </>
      }
    >
      <SimCanvas
        world={WORLD}
        draw={draw}
        deps={[T_AB, T_BC, w_C, T1, showT1]}
        aspect={12 / 9}
        padding={0.2}
        onPointer={onPointer}
        ariaLabel="Three coordinate frames A, B and C drawn as axis pairs on a unit grid with a point w, plus two dashed copies of frame B showing a body-frame and a world-frame displacement."
      />
      <div className="grid grid-cols-1 gap-x-4 gap-y-1 border-t border-fd-border px-3 py-2 font-mono text-[0.78rem] tabular-nums sm:grid-cols-2">
        <Row k={<>T<sub>AB</sub></>} val={pose(T_AB)} />
        <Row k={<>T<sub>BC</sub></>} val={pose(T_BC)} />
        <Row
          k={<>T<sub>A<span className="opacity-30">B</span></sub> T<sub><span className="opacity-30">B</span>C</sub> = T<sub>AC</sub></>}
          val={pose(T_AC)}
        />
        <Row k={<>w<sub>C</sub></>} val={pt(w_C)} />
        <Row k={<>T<sub>B<span className="opacity-30">C</span></sub> w<sub><span className="opacity-30">C</span></sub> = w<sub>B</sub></>} val={pt(worked.w_B)} />
        <Row k={<>T<sub>A<span className="opacity-30">B</span></sub> w<sub><span className="opacity-30">B</span></sub> = w<sub>A</sub></>} val={pt(worked.w_A)} />
        <Row k={<>T<sub>AB</sub> w<sub>A</sub> = w′<sub>A</sub> (displacement, no cancellation)</>} val={pt(worked.w_displaced)} />
        <Row k={<>T₁</>} val={pose(T1)} />
        <Row k={<>T<sub>AB</sub> T₁ (body) = B′</>} val={pose(worked.body)} />
        <Row k={<>T₁ T<sub>AB</sub> (world) = B″</>} val={pose(worked.world)} />
      </div>
      {showMatrices ? (
        <div className="grid grid-cols-1 gap-3 border-t border-fd-border px-3 py-2 sm:grid-cols-3">
          <Matrix name="T_AB" T={T_AB} />
          <Matrix name="T_BC" T={T_BC} />
          <Matrix name="T_AC" T={T_AC} />
        </div>
      ) : null}
      <ControlPanel columns={3}>
        <Slider
          label="T₁ heading"
          value={theta1}
          min={-Math.PI}
          max={Math.PI}
          step={Math.PI / 8}
          unit="rad"
          onChange={(val) => { sim.pause(); setManualTheta1(val); }}
          format={(val) => val.toFixed(3)}
        />
        <Slider label="T₁ translation (along y)" value={t1y} min={-2} max={2} step={0.5} onChange={setT1y} />
        <div className="flex flex-col gap-1.5">
          <Toggle label="Show B′ (body) and B″ (world)" role="path" checked={showT1} onChange={setShowT1} />
          <Toggle label="Show matrices" checked={showMatrices} onChange={setShowMatrices} />
        </div>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={() => { setManualTheta1(null); sim.toggle(); }}
        onStep={() => { setManualTheta1(null); sim.stepOnce(); }}
        onReset={() => {
          setManualTheta1(Math.PI);
          sim.pause();
          setTAB(CHOSET_FRAMES.T_AB);
          setTBC(CHOSET_FRAMES.T_BC);
          setWC(CHOSET_FRAMES.w_C);
          setT1y(CHOSET_FRAMES.T1.y);
          sim.reset();
        }}
        seed={sim.seed}
        tick={sim.tick}
      />
    </WidgetFrame>
  );
}

const fmt = (x: number) => {
  const t = tidy(x, 1e-9);
  return Number.isInteger(t) ? String(t) : t.toFixed(3);
};
const fmtAngle = (th: number) => {
  const w = wrapAngle(th);
  const k = w / (Math.PI / 4);
  if (Math.abs(k - Math.round(k)) < 1e-9) {
    const n = Math.round(k);
    if (n === 0) return '0';
    if (n === 4) return 'π';
    if (n === -4) return 'π';
    if (n === 2) return 'π/2';
    if (n === -2) return '−π/2';
    return `${n}π/4`;
  }
  return w.toFixed(3);
};

function Row({ k, val }: { k: React.ReactNode; val: string }) {
  return (
    <div className="flex items-baseline justify-between gap-3 border-b border-fd-border/60 py-0.5 last:border-b-0">
      <span className="text-fd-muted-foreground">{k}</span>
      <span>{val}</span>
    </div>
  );
}

function Matrix({ name, T }: { name: string; T: Pose2 }) {
  const M = toMatrix(T);
  return (
    <div className="font-mono text-[0.75rem] tabular-nums">
      <div className="eyebrow mb-1">{name}</div>
      <table className="border-s border-e border-fd-border">
        <tbody>
          {M.map((row, i) => (
            <tr key={i}>
              {row.map((c, j) => (
                <td key={j} className="px-2 py-0.5 text-right">{fmt(c)}</td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
