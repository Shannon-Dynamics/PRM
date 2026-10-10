import { WidgetFrame } from '@/components/sim/widget-frame';
import { PLANNING_ROLES, roleVar, type PlanningRole } from '@/lib/roles';

/**
 * w1.4 — How to Read This Book: the FCP rhythm and the seven-role color legend.
 *
 * A static figure (no client JavaScript). The strip is the order every chapter
 * follows; the legend is the color code that equation terms, figure elements,
 * and code comments share. It is reused in Appendix F.
 */

const RHYTHM: { tag: string; title: string; gloss: string }[] = [
  { tag: 'Hook', title: 'The problem', gloss: 'a widget runs before any prose' },
  { tag: 'C', title: 'Building intuition', gloss: 'play first; the mathematics is named, not yet derived' },
  { tag: 'F', title: 'The mathematics', gloss: 'definitions, theorems, derivations' },
  { tag: 'P', title: 'Implementation in Rust', gloss: 'the code the widgets run' },
  { tag: 'Lab', title: 'Putting it together', gloss: 'one run in the three worlds' },
  { tag: 'Ex', title: 'Exercises', gloss: 'derive · predict · build' },
];

const ROLE_GLOSS: Record<PlanningRole, { name: string; where: string; tex: string; code: string }> = {
  start: { name: 'start', where: 'q_start', tex: '\\htmlClass{term-start}{q_{start}}', code: '// --rm-start' },
  goal: { name: 'goal', where: 'q_goal · a collision flash', tex: '\\htmlClass{term-goal}{q_{goal}}', code: '// --rm-goal' },
  obstacle: { name: 'obstacle', where: 'WO_i, solid', tex: '\\htmlClass{term-obstacle}{WO_i}', code: '// --rm-obstacle' },
  cobstacle: { name: 'C-obstacle', where: 'QO_i, hatched', tex: '\\htmlClass{term-cobstacle}{QO_i}', code: '// --rm-cobstacle' },
  graph: { name: 'roadmap / tree', where: 'explored set, edges', tex: '\\htmlClass{term-graph}{G}', code: '// --rm-graph' },
  path: { name: 'path', where: 'the planned c(s)', tex: '\\htmlClass{term-path}{c(s)}', code: '// --rm-path' },
  robot: { name: 'robot', where: 'R(q), executed trajectory', tex: '\\htmlClass{term-robot}{R(q)}', code: '// --rm-robot' },
};

export function HowToRead() {
  const W = 720;
  const boxW = 104;
  const gap = (W - RHYTHM.length * boxW) / (RHYTHM.length - 1);

  return (
    <WidgetFrame
      id="w1.4"
      title="How to Read This Book"
      teaches="Every chapter runs the same rhythm, and every color means the same thing in a figure, an equation, and a line of Rust."
      colorKey={PLANNING_ROLES}
      caption={
        <>
          Top: the rhythm every chapter follows — a hook you can drag before a sentence of theory, the
          conceptual pass, the foundation, the Rust, an integration lab in the running worlds, and
          exercises in three flavors (F derive, C predict-then-check, P build). Bottom: the seven
          roles of the color code. A term tinted in an equation is the same object as the stroke of
          that color in the figure beside it and the <code>// --rm-*</code> comment in the listing
          below it; hovering a tinted term fades every other role on the page.
        </>
      }
    >
      <div className="overflow-x-auto p-3">
        <svg viewBox={`0 0 ${W} 92`} className="w-full" role="img" aria-label="The six-stage chapter rhythm: Hook, Conceptual, Foundation, Practical, Lab, Exercises">
          {RHYTHM.map((r, i) => {
            const x = i * (boxW + gap);
            return (
              <g key={r.tag}>
                <rect x={x} y={8} width={boxW} height={58} rx={3} style={{ fill: 'var(--color-fd-card)', stroke: 'var(--color-fd-border)' }} />
                <text x={x + 10} y={27} style={{ fill: 'var(--color-fd-primary)', fontFamily: 'ui-monospace, monospace', fontSize: 11, fontWeight: 700 }}>
                  {r.tag}
                </text>
                <text x={x + 10} y={44} style={{ fill: 'var(--color-fd-foreground)', fontFamily: 'ui-sans-serif, system-ui, sans-serif', fontSize: 11, fontWeight: 600 }}>
                  {r.title}
                </text>
                <text x={x + 10} y={59} style={{ fill: 'var(--color-fd-muted-foreground)', fontFamily: 'ui-sans-serif, system-ui, sans-serif', fontSize: 8.5 }}>
                  {r.gloss.length > 34 ? r.gloss.slice(0, 33) + '…' : r.gloss}
                </text>
                {i < RHYTHM.length - 1 ? (
                  <path d={`M ${x + boxW + 3} 37 L ${x + boxW + gap - 3} 37 M ${x + boxW + gap - 8} 32 L ${x + boxW + gap - 3} 37 L ${x + boxW + gap - 8} 42`} style={{ stroke: 'var(--color-fd-muted-foreground)', strokeWidth: 1.2, fill: 'none' }} />
                ) : null}
              </g>
            );
          })}
          <text x={0} y={86} style={{ fill: 'var(--color-fd-muted-foreground)', fontFamily: 'ui-sans-serif, system-ui, sans-serif', fontSize: 9 }}>
            Concrete before abstract: the figure comes before the equation, every time. Theorem–proof order is used only inside F.
          </text>
        </svg>
      </div>

      <div className="border-t border-fd-border">
        <table className="w-full border-collapse text-start font-ui text-[0.78rem]">
          <caption className="sr-only">The seven planning roles and how each appears in figures, equations, and code</caption>
          <thead>
            <tr className="border-b border-fd-border">
              <th scope="col" className="eyebrow px-3 py-2 text-start">Role</th>
              <th scope="col" className="eyebrow px-3 py-2 text-start">In a figure</th>
              <th scope="col" className="eyebrow px-3 py-2 text-start">In an equation</th>
              <th scope="col" className="eyebrow px-3 py-2 text-start">In Rust</th>
            </tr>
          </thead>
          <tbody>
            {PLANNING_ROLES.map((role) => {
              const g = ROLE_GLOSS[role];
              return (
                <tr key={role} className="border-b border-fd-border/60 last:border-b-0">
                  <td className="px-3 py-1.5">
                    <span className="inline-flex items-center gap-2">
                      <span aria-hidden="true" className="inline-block size-3 rounded-[2px]" style={{ backgroundColor: roleVar(role), backgroundImage: role === 'cobstacle' ? 'repeating-linear-gradient(45deg, transparent 0 2px, var(--color-fd-card) 2px 3px)' : undefined }} />
                      <span style={{ color: roleVar(role) }} className="font-medium">
                        {g.name}
                      </span>
                    </span>
                  </td>
                  <td className="px-3 py-1.5 text-fd-muted-foreground">{g.where}</td>
                  <td className="px-3 py-1.5 font-mono text-[0.7rem] text-fd-muted-foreground">{g.tex}</td>
                  <td className="px-3 py-1.5 font-mono text-[0.7rem]" style={{ color: roleVar(role) }}>
                    {g.code}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </WidgetFrame>
  );
}
