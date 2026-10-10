import { WidgetFrame } from '@/components/sim/widget-frame';
import { RETRO_COSTS, RETRO_WINS } from '@/lib/capstone/dependencies';

/**
 * w23.4 — Retrospective Scorecard (static).
 *
 * Two panels of exactly the same width and weight: on the left, five mistakes
 * the type system refused while the capstone was written; on the right, what
 * that safety cost. "Rust is free safety" is the misconception — the layout is
 * the argument.
 */
export function RetrospectiveScorecard() {
  return (
    <WidgetFrame
      id="w23.4"
      title="Retrospective Scorecard"
      teaches="Rust is not free safety: what the compiler caught and what it cost get the same amount of space."
      colorKey={['start', 'goal']}
      wide
      caption={
        <>
          Left: the diagnostic each mistake produces for the listings as printed in this chapter (abridged — paths
          and line numbers elided; the error codes are rustc&apos;s), with the contract it enforces. Right: the bill.
          No compile-time figure is printed because this edition did not measure one; the retrospective in the text
          says what measuring it would take.
        </>
      }
    >
      <div className="grid grid-cols-1 md:grid-cols-2">
        <section className="border-b border-fd-border p-3 md:border-b-0 md:border-r" aria-label="What the type system caught">
          <p className="eyebrow mb-2" style={{ color: 'var(--rm-start)' }}>
            A · caught at compile time
          </p>
          <ol className="space-y-3">
            {RETRO_WINS.map((w, i) => (
              <li key={w.title}>
                <p className="font-ui text-[0.78rem] font-semibold">
                  {i + 1}. {w.title}
                </p>
                <pre className="mt-1 overflow-x-auto rounded-sm border border-fd-border bg-fd-muted/40 px-2 py-1 font-mono text-[0.66rem] leading-snug">
                  <code>{w.code}</code>
                </pre>
                <pre className="mt-1 overflow-x-auto whitespace-pre-wrap px-2 font-mono text-[0.64rem] leading-snug" style={{ color: 'var(--rm-goal)' }}>
                  {w.error}
                </pre>
                <p className="mt-1 font-ui text-[0.72rem] leading-snug text-fd-muted-foreground">{w.moral}</p>
              </li>
            ))}
          </ol>
        </section>
        <section className="p-3" aria-label="What it cost">
          <p className="eyebrow mb-2" style={{ color: 'var(--rm-goal)' }}>
            B · paid for it
          </p>
          <ol className="space-y-3">
            {RETRO_COSTS.map((c, i) => (
              <li key={c.title}>
                <p className="font-ui text-[0.78rem] font-semibold">
                  {i + 1}. {c.title}
                </p>
                <p className="mt-1 font-ui text-[0.72rem] leading-snug text-fd-muted-foreground">{c.detail}</p>
              </li>
            ))}
          </ol>
        </section>
      </div>
    </WidgetFrame>
  );
}
