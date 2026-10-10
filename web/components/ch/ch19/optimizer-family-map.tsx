'use client';

import { useState } from 'react';

/**
 * f19.4 — the Optimizer Family Map. A static SVG with hover: the chapter's
 * methods placed in one design space rather than listed as a zoo. Horizontal
 * axis: what drives the step (first-order gradient, second-order model,
 * sampling). Vertical: how obstacles enter (as a cost, as a constraint). Marker
 * shape: shooting (parameterize u, integrate) versus collocation (parameterize
 * q, differentiate). MPC and shooting wrap the others.
 */

interface Node {
  id: string;
  x: number;
  y: number;
  label: string;
  shape: 'shooting' | 'collocation' | 'wrapper';
  note: string;
  where: string;
}

const NODES: Node[] = [
  { id: 'shooting', x: 18, y: 34, label: 'Shooting (C §11.3.1)', shape: 'shooting', note: 'Guess λ(0), integrate state and adjoint with ∂H/∂u = 0, correct the guess from the terminal miss. The initial-guess problem is explicit here first.', where: 'second-order model of the terminal map; no obstacles' },
  { id: 'sqp', x: 52, y: 78, label: 'Transcription + SQP (C §11.3.2)', shape: 'collocation', note: 'Parameterize q(t) or u(t), sample the constraints at k + 1 points, hand J(X), g(X) and their gradients to a sequential quadratic programmer.', where: 'second-order; obstacles as constraints h(q) ≤ 0' },
  { id: 'chomp', x: 22, y: 22, label: 'CHOMP (2009, 2013)', shape: 'collocation', note: 'Covariant functional gradient over a distance field: ξ ← ξ − (1/η) A⁻¹∇F. Obstacles are a C¹ cost c(D); clearance is bought with λ and ε, so collisions can survive convergence.', where: 'first-order in the smoothness metric; obstacles as cost' },
  { id: 'stomp', x: 86, y: 22, label: 'STOMP (2011)', shape: 'collocation', note: 'K seeded rollouts with noise N(0, A⁻¹), exponentiated cost-weighted update, smoothed by A⁻¹. No gradient at all, so any evaluable cost will do.', where: 'sampling; obstacles as cost' },
  { id: 'trajopt', x: 52, y: 60, label: 'TrajOpt (2013, 2014)', shape: 'collocation', note: 'Sequential convex optimization: convexify, solve a QP in an ℓ∞ trust region, raise an ℓ₁ penalty until sd(q) ≥ d_safe holds exactly. Our `sco` keeps the shape with LM in place of the QP.', where: 'second-order (QP); obstacles as constraints' },
  { id: 'ilqr', x: 56, y: 26, label: 'iLQR / DDP (1970, 2004, 2012)', shape: 'shooting', note: 'Backward quadratic value recursion along a nominal rollout, closed-loop forward pass with line search. DDP keeps f_xx; iLQR drops it. On an LQ problem one pass is Riccati.', where: 'second-order value model; obstacles as cost (soft)' },
  { id: 'mpc', x: 56, y: 10, label: 'MPC (the loop)', shape: 'wrapper', note: 'Each tick: solve a horizon-N_h problem from the measured state, apply u₀, discard the rest, warm-start the next. A wrapper around any solver that accepts a warm start.', where: 'wraps iLQR here; the sister book wraps MPPI' },
  { id: 'mppi', x: 86, y: 40, label: 'MPPI (sister Ch. 23)', shape: 'shooting', note: "Sampling-based MPC: roll out K noisy control sequences, weight by exp(−S/λ), average. STOMP's idea on controls, inside the MPC loop.", where: 'sampling; shooting; obstacles as cost' },
];

const SHAPE_NOTE: Record<Node['shape'], string> = {
  shooting: 'shooting: parameterize u(t) and integrate the dynamics',
  collocation: 'collocation: parameterize q(t) and read u from the dynamics',
  wrapper: 'a loop around another solver',
};

export function OptimizerFamilyMap() {
  const [hover, setHover] = useState<string | null>('chomp');
  const active = NODES.find((n) => n.id === hover) ?? null;
  return (
    <figure id="f19.4" className="not-prose my-8" aria-labelledby="f19.4-title">
      <div className="overflow-hidden rounded-md border border-fd-border bg-fd-card">
        <div className="flex flex-wrap items-baseline gap-x-3 gap-y-1 border-b border-fd-border px-3 py-2">
          <span className="font-mono text-[0.7rem] font-medium text-fd-primary">f19.4</span>
          <span id="f19.4-title" className="font-display text-sm font-semibold">The Optimizer Family Map</span>
          <span className="w-full font-ui text-xs text-fd-muted-foreground sm:w-auto sm:flex-1">These are not competing algorithms; they are points in one design space.</span>
        </div>
        <svg viewBox="0 0 100 92" className="block w-full" role="img" aria-label="A map of trajectory optimizers on two axes: what drives the step (first-order, second-order, sampling) and how obstacles enter (cost or constraint).">
          <defs>
            <marker id="f194-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse">
              <path d="M 0 0 L 10 5 L 0 10 z" fill="var(--pr-canvas-ink, currentColor)" />
            </marker>
          </defs>
          <line x1="8" y1="88" x2="96" y2="88" stroke="var(--pr-canvas-ink, currentColor)" strokeWidth="0.3" markerEnd="url(#f194-arrow)" />
          <line x1="8" y1="88" x2="8" y2="4" stroke="var(--pr-canvas-ink, currentColor)" strokeWidth="0.3" markerEnd="url(#f194-arrow)" />
          <text x="22" y="91.5" fontSize="2.4" fill="var(--pr-canvas-ink, currentColor)" fontFamily="ui-monospace, monospace">first-order gradient</text>
          <text x="48" y="91.5" fontSize="2.4" fill="var(--pr-canvas-ink, currentColor)" fontFamily="ui-monospace, monospace">second-order model</text>
          <text x="80" y="91.5" fontSize="2.4" fill="var(--pr-canvas-ink, currentColor)" fontFamily="ui-monospace, monospace">sampling</text>
          <text x="2" y="24" fontSize="2.4" fill="var(--pr-canvas-ink, currentColor)" fontFamily="ui-monospace, monospace" transform="rotate(-90 2 24)">obstacles as cost</text>
          <text x="2" y="78" fontSize="2.4" fill="var(--pr-canvas-ink, currentColor)" fontFamily="ui-monospace, monospace" transform="rotate(-90 2 78)">as constraint</text>
          <line x1="9" y1="48" x2="95" y2="48" stroke="var(--pr-grid, currentColor)" strokeWidth="0.3" strokeDasharray="1 1" />
          {/* lineage arrows */}
          <path d="M 24 34 C 34 36, 44 30, 54 27" fill="none" stroke="var(--rm-graph)" strokeWidth="0.35" strokeDasharray="1.2 0.8" />
          <path d="M 52 76 C 52 72, 52 66, 52 62" fill="none" stroke="var(--rm-graph)" strokeWidth="0.35" strokeDasharray="1.2 0.8" />
          <path d="M 28 22 C 48 16, 66 16, 84 21" fill="none" stroke="var(--rm-graph)" strokeWidth="0.35" strokeDasharray="1.2 0.8" />
          <path d="M 56 24 C 56 20, 56 16, 56 12" fill="none" stroke="var(--rm-graph)" strokeWidth="0.35" strokeDasharray="1.2 0.8" />
          <path d="M 84 24 C 88 28, 88 34, 86 38" fill="none" stroke="var(--rm-graph)" strokeWidth="0.35" strokeDasharray="1.2 0.8" />
          {NODES.map((n) => {
            const on = hover === n.id;
            const fill = n.shape === 'wrapper' ? 'var(--rm-robot)' : n.shape === 'shooting' ? 'var(--rm-path)' : 'var(--rm-graph)';
            return (
              <g key={n.id} onMouseEnter={() => setHover(n.id)} onFocus={() => setHover(n.id)} tabIndex={0} style={{ cursor: 'pointer', outline: 'none' }}>
                <title>{n.label}</title>
                {n.shape === 'collocation' ? (
                  <rect x={n.x - 1.6} y={n.y - 1.6} width={3.2} height={3.2} fill={fill} opacity={on ? 1 : 0.75} stroke={on ? 'var(--pr-canvas-ink, currentColor)' : 'none'} strokeWidth={0.4} />
                ) : n.shape === 'shooting' ? (
                  <circle cx={n.x} cy={n.y} r={1.7} fill={fill} opacity={on ? 1 : 0.75} stroke={on ? 'var(--pr-canvas-ink, currentColor)' : 'none'} strokeWidth={0.4} />
                ) : (
                  <path d={`M ${n.x} ${n.y - 2.1} L ${n.x + 2.1} ${n.y} L ${n.x} ${n.y + 2.1} L ${n.x - 2.1} ${n.y} Z`} fill={fill} opacity={on ? 1 : 0.75} stroke={on ? 'var(--pr-canvas-ink, currentColor)' : 'none'} strokeWidth={0.4} />
                )}
                <text x={n.x + 2.6} y={n.y + 0.9} fontSize="2.3" fontWeight={on ? 700 : 500} fill="var(--pr-canvas-ink, currentColor)" fontFamily="ui-monospace, monospace">
                  {n.label}
                </text>
              </g>
            );
          })}
        </svg>
        <div className="border-t border-fd-border px-3 py-2 font-ui text-xs leading-relaxed text-fd-muted-foreground" style={{ minHeight: '4.5rem' }}>
          {active ? (
            <>
              <span className="font-semibold text-fd-foreground">{active.label}</span> — {active.note} <span className="italic">({active.where}; {SHAPE_NOTE[active.shape]}.)</span>
            </>
          ) : (
            'Hover or focus a node.'
          )}
        </div>
      </div>
      <figcaption className="pr-caption mt-2 font-ui text-[0.8rem] leading-relaxed text-fd-muted-foreground">
        Squares are collocation methods (the trajectory is the variable), circles are shooting methods (the controls are), the diamond is a loop
        around any of them. Dashed blue arrows are lineage: Pontryagin&apos;s shooting to iLQR&apos;s value recursion, transcription to TrajOpt&apos;s
        sequential convexification, CHOMP&apos;s smoothness metric to STOMP&apos;s noise covariance, and STOMP&apos;s rollouts to MPPI inside the MPC loop.
      </figcaption>
    </figure>
  );
}
