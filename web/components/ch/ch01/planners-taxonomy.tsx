'use client';

import { useState } from 'react';
import Link from 'next/link';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { ALL_CHAPTERS } from '@/lib/book-structure';

/**
 * w1.3 — The Planner's Taxonomy: Choset's Table 1.1 as a board.
 *
 * Three columns — task, robot, algorithm — and under them the twenty-three
 * chapters. Hover (or focus) a cell and the chapters that live in it light up.
 * The misconception killed: "motion planning means finding the shortest path".
 * Most of the board is not about optimality at all.
 */

interface Cell {
  id: string;
  label: string;
  gloss: string;
  chapters: number[];
}

const COLUMNS: { title: string; cells: Cell[] }[] = [
  {
    title: 'Task',
    cells: [
      { id: 'navigate', label: 'Navigate', gloss: 'a collision-free motion from one configuration to another', chapters: [1, 3, 6, 7, 8, 9, 11, 12, 13, 14, 21, 23] },
      { id: 'cover', label: 'Cover', gloss: 'pass a sensor or tool over every point of a region', chapters: [10] },
      { id: 'localize', label: 'Localize', gloss: 'use a map and sensor data to find the configuration', chapters: [15, 16] },
      { id: 'map', label: 'Map', gloss: 'explore an unknown environment into a representation', chapters: [9, 16] },
    ],
  },
  {
    title: 'Robot',
    cells: [
      { id: 'cspace', label: 'Configuration space, DOF', gloss: 'how many numbers, and on what manifold', chapters: [2, 4, 5, 14] },
      { id: 'omni', label: 'Omnidirectional', gloss: 'free to move instantaneously in any direction of Q — Rusty as a disc, the sofa', chapters: [3, 7, 8, 11, 12, 13] },
      { id: 'nonholo', label: 'Nonholonomic', gloss: 'velocity constraints: Hitch cannot slide sideways', chapters: [14, 20, 21, 23] },
      { id: 'kinematic', label: 'Kinematic', gloss: 'velocities as controls', chapters: [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16] },
      { id: 'dynamic', label: 'Dynamic', gloss: 'forces as controls; velocities become state', chapters: [17, 18, 19] },
    ],
  },
  {
    title: 'Algorithm',
    cells: [
      { id: 'optimal', label: 'Optimal', gloss: 'shortest, fastest, or cheapest — a bounded-cost promise', chapters: [6, 8, 13, 18, 19, 21] },
      { id: 'feasible', label: 'Non-optimal (feasible)', gloss: 'any solution satisfying the constraints', chapters: [3, 7, 11, 12, 14] },
      { id: 'complexity', label: 'Computational complexity', gloss: 'polynomial or exponential in the size of the problem', chapters: [6, 9, 11] },
      { id: 'complete', label: 'Complete', gloss: 'finds a solution whenever one exists, or reports failure in finite time', chapters: [3, 6, 8, 9, 10] },
      { id: 'rescomplete', label: 'Resolution complete', gloss: 'complete at the chosen discretization', chapters: [6, 7, 10, 21] },
      { id: 'probcomplete', label: 'Probabilistically complete', gloss: 'finds a solution with probability → 1 as time grows', chapters: [11, 12, 13, 14, 22] },
      { id: 'online', label: 'Online / sensor-based', gloss: 'interleaves sensing, computation, and action', chapters: [3, 9, 10, 16, 23] },
      { id: 'offline', label: 'Offline / world model', gloss: 'plans from a known model before executing', chapters: [6, 7, 8, 10, 11, 12, 13, 19] },
    ],
  },
];

export function PlannersTaxonomy() {
  const [active, setActive] = useState<Cell | null>(null);
  const lit = new Set(active?.chapters ?? []);

  return (
    <WidgetFrame
      id="w1.3"
      title="The Planner's Taxonomy"
      teaches="Motion planning is not 'find the shortest path'. Most of the board has nothing to do with optimality."
      colorKey={['graph']}
      caption={
        <>
          Choset's Table 1.1: a motion planner is characterized by the <em>task</em> it solves, the
          <em> robot</em> it solves it for, and the properties of the <em>algorithm</em>. Hover or
          focus a cell to light the chapters that live in it. Notice how few chapters sit in
          "Optimal" — and that "sensor-based + navigate + complete" is a cell with real theorems in it
          (Chapters 3 and 9), not a compromise.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border sm:grid-cols-3">
        {COLUMNS.map((col) => (
          <div key={col.title} className="bg-fd-card p-3">
            <p className="eyebrow mb-2">{col.title}</p>
            <ul className="flex flex-col gap-1" role="list">
              {col.cells.map((c) => {
                const on = active?.id === c.id;
                return (
                  <li key={c.id}>
                    <button
                      type="button"
                      onMouseEnter={() => setActive(c)}
                      onFocus={() => setActive(c)}
                      onClick={() => setActive(on ? null : c)}
                      aria-pressed={on}
                      className="w-full rounded-sm border px-2 py-1 text-start font-ui text-[0.78rem] transition-colors hover:bg-fd-accent focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-fd-primary"
                      style={on ? { borderColor: 'var(--rm-graph)', color: 'var(--rm-graph)' } : { borderColor: 'var(--color-fd-border)' }}
                    >
                      {c.label}
                    </button>
                  </li>
                );
              })}
            </ul>
          </div>
        ))}
      </div>

      <div className="border-t border-fd-border px-3 py-2">
        <p className="min-h-[2.4em] font-ui text-[0.78rem] text-fd-muted-foreground">
          {active ? (
            <>
              <span className="font-medium text-fd-foreground">{active.label}:</span> {active.gloss}.{' '}
              <span className="font-mono text-[0.7rem]">
                {active.chapters.length} chapter{active.chapters.length === 1 ? '' : 's'}
              </span>
            </>
          ) : (
            'Point at a cell to see which chapters occupy it.'
          )}
        </p>
        <ol className="mt-2 flex flex-wrap gap-1" role="list" aria-label="Chapters">
          {ALL_CHAPTERS.map((ch) => {
            const on = lit.has(ch.n);
            return (
              <li key={ch.n}>
                <Link
                  href={`/chapters/${ch.slug}`}
                  title={ch.title}
                  className="inline-flex size-7 items-center justify-center rounded-sm border font-mono text-[0.7rem] tabular-nums transition-colors"
                  style={
                    on
                      ? { borderColor: 'var(--rm-graph)', backgroundColor: 'var(--rm-graph)', color: 'var(--color-fd-primary-foreground)' }
                      : { borderColor: 'var(--color-fd-border)', opacity: active ? 0.45 : 1 }
                  }
                >
                  {ch.n}
                </Link>
              </li>
            );
          })}
        </ol>
      </div>
    </WidgetFrame>
  );
}
