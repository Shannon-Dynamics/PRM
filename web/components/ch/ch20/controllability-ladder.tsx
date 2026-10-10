'use client';

import { useState } from 'react';

/**
 * w20.4 — the Controllability Ladder: Choset's Figure 12.12 with its conditional
 * arrows annotated. Solid arrows are implications that always hold; dashed
 * ones need the condition written on them. Hover (or tab to) a node for its
 * definition, the counterexample that separates it from its neighbors, and the
 * lab system that exhibits it.
 */

interface Node {
  id: string;
  x: number;
  y: number;
  label: string;
  def: string;
  example: string;
  lab: string;
}

const NODES: Node[] = [
  {
    id: 'acc',
    x: 22,
    y: 22,
    label: 'accessible',
    def: 'R^M(x, ≤ T) contains a full n-dimensional set for some T — possibly only after leaving every small neighborhood of x.',
    example: 'Choset Figure 12.11(a): the reachable set has interior, but not while confined to V. Any STLA system is accessible; the converse needs analytic fields.',
    lab: 'Every system in the lab with a full LARC is accessible; the property is too weak to plan with.',
  },
  {
    id: 'ctrl',
    x: 78,
    y: 22,
    label: 'controllable',
    def: 'Every x_goal ∈ M is reachable from x in finite time, with no restriction on where the trajectory goes meanwhile.',
    example: 'The watch knob that turns one way: controllable (go round) but not STLC (cannot turn back a minute). The WPPS example ẋ = ½[−x₂, x₁] + e₁u is controllable everywhere and STLC only at the origin.',
    lab: 'Hitch driving forward only (the Dubins car) in the empty Lot: controllable, STLA, not STLC — the lemma Dubins needs in Chapter 21.',
  },
  {
    id: 'stla',
    x: 22,
    y: 74,
    label: 'small-time locally accessible',
    def: 'R^V(x, ≤ T) has interior for every neighborhood V and every T > 0: the LARC, Lie{g₀,…,g_m}(x) = T_xM.',
    example: 'ẋ = [x₂², 0]ᵀ + [0, 1]ᵀu: STLA everywhere, never controllable — x₁ only ever grows. The double integrator at nonzero velocity: STLA, not STLC.',
    lab: 'The PBWT with a single offset thruster (Reach\'s free third link pushed at one point): det = −16d⁸, STLA and controllable, bad brackets not neutralized.',
  },
  {
    id: 'stlc',
    x: 78,
    y: 74,
    label: 'small-time locally controllable',
    def: 'R^V(x, ≤ T) contains a neighborhood of x for every V and T: the system can wiggle in every direction without leaving any ball.',
    example: 'Symmetric systems with the LARC (Chow); systems with drift at an equilibrium when every bad bracket is neutralized (Sussmann).',
    lab: 'Hitch with and without the trailer (ladder 2→3→4(→5)); the PBWT with two thrusters at rest; Reach\'s 2R on the Workbench.',
  },
];

interface Arrow {
  from: string;
  to: string;
  cond?: string;
  /** Label anchor. */
  lx: number;
  ly: number;
}

const ARROWS: Arrow[] = [
  { from: 'ctrl', to: 'acc', lx: 50, ly: 17 },
  { from: 'stla', to: 'acc', lx: 10, ly: 48 },
  { from: 'stlc', to: 'stla', lx: 50, ly: 70 },
  { from: 'acc', to: 'stla', cond: 'if all vector fields are analytic', lx: 34, ly: 48 },
  { from: 'stla', to: 'stlc', cond: 'if symmetric (driftless, U ∈ U±)', lx: 50, ly: 83 },
  { from: 'stlc', to: 'ctrl', cond: 'if M is connected and STLC holds everywhere', lx: 92, ly: 48 },
];

const pos = (id: string) => NODES.find((n) => n.id === id) as Node;

export function ControllabilityLadder() {
  const [hover, setHover] = useState<string | null>('stlc');
  const active = NODES.find((n) => n.id === hover) ?? null;
  const ink = 'var(--pr-canvas-ink, currentColor)';

  return (
    <figure id="w20.4" className="not-prose my-8" aria-labelledby="w20.4-title">
      <div className="overflow-hidden rounded-md border border-fd-border bg-fd-card">
        <div className="flex flex-wrap items-baseline gap-x-3 gap-y-1 border-b border-fd-border px-3 py-2">
          <span className="font-mono text-[0.7rem] font-medium text-fd-primary">w20.4</span>
          <span id="w20.4-title" className="font-display text-sm font-semibold">Controllability Ladder</span>
          <span className="w-full font-ui text-xs text-fd-muted-foreground sm:w-auto sm:flex-1">Four properties, three unconditional implications, three conditional ones — and no Kalman rank test anywhere.</span>
        </div>
        <div className="grid gap-0 md:grid-cols-[minmax(0,3fr)_minmax(0,2fr)]">
          <svg viewBox="0 0 100 96" className="block w-full" role="img" aria-label="Choset's implication diagram among accessible, controllable, small-time locally accessible and small-time locally controllable, with conditional arrows labelled.">
            <defs>
              <marker id="w204-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse">
                <path d="M 0 0 L 10 5 L 0 10 z" fill={ink} />
              </marker>
              <marker id="w204-arrow-blue" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse">
                <path d="M 0 0 L 10 5 L 0 10 z" fill="var(--rm-graph)" />
              </marker>
            </defs>
            {ARROWS.map((a) => {
              const f = pos(a.from);
              const t = pos(a.to);
              const dx = t.x - f.x;
              const dy = t.y - f.y;
              const len = Math.hypot(dx, dy);
              const ux = dx / len;
              const uy = dy / len;
              // Offset parallel pairs (acc↔stla, stla↔stlc, ctrl↔stlc) so both directions are visible.
              const side = a.cond ? 1 : -1;
              const ox = -uy * 2.2 * side;
              const oy = ux * 2.2 * side;
              const pad = 9;
              const x1 = f.x + ux * pad + ox;
              const y1 = f.y + uy * pad + oy;
              const x2 = t.x - ux * pad + ox;
              const y2 = t.y - uy * pad + oy;
              return (
                <g key={`${a.from}-${a.to}`}>
                  <line x1={x1} y1={y1} x2={x2} y2={y2} stroke={a.cond ? 'var(--rm-graph)' : ink} strokeWidth={0.5} strokeDasharray={a.cond ? '1.6 1.2' : undefined} markerEnd={a.cond ? 'url(#w204-arrow-blue)' : 'url(#w204-arrow)'} />
                  {a.cond ? (
                    <text x={a.lx} y={a.ly} fontSize="2.3" fill="var(--rm-graph)" textAnchor="middle" fontFamily="ui-monospace, monospace">
                      {a.cond}
                    </text>
                  ) : null}
                </g>
              );
            })}
            {NODES.map((n) => {
              const on = hover === n.id;
              return (
                <g key={n.id} onMouseEnter={() => setHover(n.id)} onFocus={() => setHover(n.id)} tabIndex={0} style={{ cursor: 'pointer', outline: 'none' }}>
                  <title>{n.label}</title>
                  <rect x={n.x - 17} y={n.y - 7} width={34} height={14} rx={1.2} fill={on ? 'var(--rm-graph)' : 'var(--rm-graph)'} opacity={on ? 0.95 : 0.18} stroke="var(--rm-graph)" strokeWidth={0.4} />
                  <text x={n.x} y={n.y - 1} fontSize="2.6" textAnchor="middle" fill={on ? 'var(--fd-primary-foreground, #fff)' : ink} fontFamily="ui-monospace, monospace" fontWeight={600}>
                    {n.label.split(' ').slice(0, 2).join(' ')}
                  </text>
                  {n.label.split(' ').length > 2 ? (
                    <text x={n.x} y={n.y + 3.2} fontSize="2.6" textAnchor="middle" fill={on ? 'var(--fd-primary-foreground, #fff)' : ink} fontFamily="ui-monospace, monospace" fontWeight={600}>
                      {n.label.split(' ').slice(2).join(' ')}
                    </text>
                  ) : null}
                </g>
              );
            })}
            <text x="50" y="93" fontSize="2.2" textAnchor="middle" fill={ink} fontFamily="ui-monospace, monospace">solid: always · dashed (blue): under the stated condition · after Choset, Figure 12.12</text>
          </svg>
          <div className="border-t border-fd-border p-3 font-ui text-xs leading-relaxed md:border-s md:border-t-0">
            {active ? (
              <>
                <p className="font-display text-sm font-semibold">{active.label}</p>
                <p className="mt-1.5 text-fd-muted-foreground">{active.def}</p>
                <p className="mt-2"><span className="eyebrow">separating example</span></p>
                <p className="text-fd-muted-foreground">{active.example}</p>
                <p className="mt-2"><span className="eyebrow">in the lab</span></p>
                <p className="text-fd-muted-foreground">{active.lab}</p>
              </>
            ) : (
              <p className="text-fd-muted-foreground">Hover a node.</p>
            )}
          </div>
        </div>
      </div>
      <figcaption className="pr-caption mt-2 font-ui text-[0.8rem] leading-relaxed text-fd-muted-foreground">
        The linearization of every nonholonomic system in this chapter fails the Kalman rank test, so none of these properties can be read off a linear model; the Lie algebra is the only tool, and even it decides STLC only under the conditions on the dashed arrows.
      </figcaption>
    </figure>
  );
}
