import { WidgetFrame } from '@/components/sim/widget-frame';
import { LIBRARY_MAP } from '@/lib/capstone/dependencies';

/**
 * f23.2 — fig-23-library-map (static).
 *
 * Each stage of the capstone beside the component that plays the same role in
 * OMPL, MoveIt and Nav2. The point is the shape, not the names: the same
 * contracts (state space, validity checker, planner, post-processor, time
 * parameterization, controller, recovery) recur in every production stack.
 * Names are dated October 2026 and rot fastest of anything in the book.
 */
export function LibraryMap() {
  return (
    <WidgetFrame
      id="f23.2"
      title="fig-23-library-map"
      teaches="Our stages are not toys with no relatives: each contract has a production counterpart, and a dash where a library deliberately has none."
      colorKey={['path']}
      wide
      caption={
        <>
          Stages of this chapter&apos;s three pipelines and their counterparts as the projects&apos; documentation
          named them in October 2026. A dash means the library does not own that job — OMPL is a planning library with
          no map or controller, Nav2 has no time parameterization because its controllers emit velocities directly.
          Check the current documentation before relying on a name.
        </>
      }
    >
      <div className="overflow-x-auto">
        <table className="w-full min-w-[640px] border-collapse font-ui text-[0.74rem]">
          <thead>
            <tr className="border-b border-fd-border text-left">
              <th className="px-3 py-2 font-semibold">Stage</th>
              <th className="px-3 py-2 font-semibold" style={{ color: 'var(--rm-path)' }}>
                This book
              </th>
              <th className="px-3 py-2 font-semibold">OMPL</th>
              <th className="px-3 py-2 font-semibold">MoveIt</th>
              <th className="px-3 py-2 font-semibold">Nav2</th>
            </tr>
          </thead>
          <tbody>
            {LIBRARY_MAP.map((r) => (
              <tr key={r.stage} className="border-b border-fd-border last:border-b-0 align-top">
                <td className="px-3 py-1.5 font-medium">{r.stage}</td>
                <td className="px-3 py-1.5 font-mono text-[0.68rem]" style={{ color: 'var(--rm-path)' }}>
                  {r.chapter}
                </td>
                {[r.ompl, r.moveit, r.nav2].map((cell, i) => (
                  <td key={i} className={`px-3 py-1.5 ${cell.startsWith('—') ? 'text-fd-muted-foreground' : ''}`}>
                    {cell}
                  </td>
                ))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </WidgetFrame>
  );
}
