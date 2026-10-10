'use client';

import { useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';

/**
 * f14.5 — Pointer box: where Choset §7.5.4–7.5.6 went. Three static panels;
 * hovering or focusing one reveals a one-paragraph summary. No simulation:
 * these are the topics the chapter condenses rather than teaches.
 */

interface Panel {
  key: string;
  title: string;
  section: string;
  gloss: string;
  body: string;
}

const PANELS: Panel[] = [
  {
    key: 'assembly',
    title: 'Assembly planning',
    section: '§7.5.4',
    gloss: 'one subassembly is the robot, the rest is the workspace',
    body: 'An assembly operation merges two separated subassemblies; cast as path planning, one subassembly is the moving body and the other the obstacle, and PRMs, ESTs and RRTs find the merging — or, run backward, the removal — path. The α-shaped test pieces of figure 7.23 are the field\'s standard narrow-passage benchmark. Beyond planning, the NonDirectional Blocking Graph records every blocking relation in a polygonal assembly and yields the assembly order analytically. Everything in this chapter applies unchanged: the composite space is the product of the parts\' SE(3)s, and the price is the same exponential.',
  },
  {
    key: 'flexible',
    title: 'Flexible objects',
    section: '§7.5.5',
    gloss: 'a configuration space of infinite dimension, cut down by energy',
    body: 'Sheets of metal, bending pipes, cables, catheters and drug molecules have configuration spaces of potentially infinite dimension, so the first problem is a compact parametrization — low-dimensional curve representations that enforce constant length, or local deformation fields. The second is that a path must be energetically feasible as well as collision-free: roadmaps are built from quasi-static, low-energy configurations found by energy minimization or physics simulation, and every edge keeps the elastic energy under a threshold. Collision checking cannot be preprocessed because the shape keeps changing, which is the practical bottleneck.',
  },
  {
    key: 'biology',
    title: 'Biological applications',
    section: '§7.5.6',
    gloss: 'a molecule is a tree-like articulated robot',
    body: 'A molecule\'s bond graph is a tree with an anchor atom as root; with bond lengths and angles fixed, the dihedral angles are its joint variables, and a protein is an articulated robot with a few hundred of them. PRM roadmaps over this torus accept a configuration with a probability that falls with its potential energy, connect neighbors by linear interpolation checked for feasible energy along the way, and weight edges by the difficulty of the transition. Folding pathways, protein–ligand docking and Stochastic Roadmap Simulation all ride on this one reduction: the configuration-space abstraction lets problems from another science be cast as motion planning.',
  },
];

export function PointerBox() {
  const [open, setOpen] = useState<string>(PANELS[0].key);
  const active = PANELS.find((p) => p.key === open) ?? PANELS[0];

  return (
    <WidgetFrame
      id="f14.5"
      title="Pointer box: assembly, flexible objects, biology"
      teaches="Three of Choset's §7.5 extensions are condensed here, not taught: each is Part III's machinery on a configuration space with a different shape."
      caption={<>Hover or focus a panel for a one-paragraph summary. Each topic is the same idea as this chapter — a product or a quotient of configuration spaces, a free-space predicate that is expensive, and sampling to avoid building it — with a different cost model.</>}
    >
      <div className="grid grid-cols-1 gap-3 p-3 sm:grid-cols-3" role="list">
        {PANELS.map((p) => (
          <button
            key={p.key}
            type="button"
            role="listitem"
            onMouseEnter={() => setOpen(p.key)}
            onFocus={() => setOpen(p.key)}
            onClick={() => setOpen(p.key)}
            aria-pressed={open === p.key}
            className={`rounded-sm border px-3 py-3 text-left transition-colors focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-fd-primary ${open === p.key ? 'border-fd-primary bg-fd-primary/5' : 'border-fd-border bg-fd-card hover:bg-fd-accent'}`}
          >
            <svg viewBox="0 0 120 54" className="mb-2 h-14 w-full" aria-hidden="true">
              {p.key === 'assembly' ? (
                <>
                  <path d="M10 40 L10 14 L40 14 L40 24 L22 24 L22 40 Z" fill="var(--rm-obstacle)" opacity="0.8" />
                  <path d="M60 40 L60 24 L78 24 L78 14 L108 14 L108 40 Z" fill="var(--rm-robot)" opacity="0.85" />
                  <path d="M44 27 L56 27" stroke="var(--rm-path)" strokeWidth="2" strokeDasharray="3 2" />
                  <path d="M52 23 L57 27 L52 31" fill="none" stroke="var(--rm-path)" strokeWidth="2" />
                </>
              ) : p.key === 'flexible' ? (
                <>
                  <rect x="46" y="8" width="28" height="38" fill="none" stroke="var(--rm-obstacle)" strokeWidth="3" />
                  <rect x="52" y="8" width="16" height="38" fill="var(--rm-obstacle)" opacity="0.15" />
                  <path d="M8 27 C 30 27, 40 12, 60 20 S 90 34, 112 27" fill="none" stroke="var(--rm-robot)" strokeWidth="4" strokeLinecap="round" />
                </>
              ) : (
                <>
                  <path d="M14 40 L30 28 L46 36 L60 20 L78 28 L92 14 L106 24" fill="none" stroke="var(--rm-graph)" strokeWidth="2.5" />
                  {[
                    [14, 40],
                    [30, 28],
                    [46, 36],
                    [60, 20],
                    [78, 28],
                    [92, 14],
                    [106, 24],
                  ].map(([x, y], i) => (
                    <circle key={i} cx={x} cy={y} r="4.5" fill={i % 2 ? 'var(--rm-robot)' : 'var(--rm-graph)'} />
                  ))}
                  <circle cx="60" cy="20" r="7" fill="none" stroke="var(--rm-cobstacle)" strokeWidth="2" strokeDasharray="2 2" />
                </>
              )}
            </svg>
            <div className="flex items-baseline gap-2">
              <span className="font-mono text-[0.65rem] text-fd-primary">{p.section}</span>
              <span className="font-display text-sm font-semibold">{p.title}</span>
            </div>
            <p className="mt-1 font-ui text-xs text-fd-muted-foreground">{p.gloss}</p>
          </button>
        ))}
      </div>
      <div className="border-t border-fd-border px-4 py-3 font-prose text-[0.9rem] leading-relaxed" aria-live="polite">
        <span className="font-mono text-[0.7rem] text-fd-primary">{active.section} · </span>
        {active.body}
      </div>
    </WidgetFrame>
  );
}
