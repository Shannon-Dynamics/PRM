import { PLANNING_ROLES, type Role } from '@/lib/roles';

export type ColorRole = Role;

export interface ColorKeyProps {
  items?: readonly ColorRole[];
}

/**
 * The book's color code. Values are CSS vars, never literals, because each one
 * is redefined for dark mode in global.css.
 *
 * Planning roles are this book's own (TOC.md §2). The estimation roles are
 * imported unchanged from the sister book for the chapters where a belief
 * appears (Chapters 15–16, the capstone's belief panel).
 */
const ROLES: Record<ColorRole, { label: string; swatch: string; gloss: string }> = {
  start: { label: 'Start', swatch: 'var(--rm-start)', gloss: 'start configuration q_start' },
  goal: { label: 'Goal', swatch: 'var(--rm-goal)', gloss: 'goal configuration q_goal' },
  obstacle: { label: 'Obstacle', swatch: 'var(--rm-obstacle)', gloss: 'workspace obstacle' },
  cobstacle: { label: 'C-obstacle', swatch: 'var(--rm-cobstacle)', gloss: 'configurations in collision' },
  graph: { label: 'Roadmap / tree', swatch: 'var(--rm-graph)', gloss: 'explored set, roadmap, or tree' },
  path: { label: 'Path', swatch: 'var(--rm-path)', gloss: 'the solution path' },
  robot: { label: 'Robot', swatch: 'var(--rm-robot)', gloss: 'the robot and its executed trajectory' },
  prior: { label: 'Prior', swatch: 'var(--pr-prior)', gloss: 'belief before this step' },
  prediction: { label: 'Prediction', swatch: 'var(--pr-prediction)', gloss: 'belief after motion' },
  measurement: { label: 'Measurement', swatch: 'var(--pr-measurement)', gloss: 'observation likelihood' },
  posterior: { label: 'Posterior', swatch: 'var(--pr-posterior)', gloss: 'belief after the update' },
  truth: { label: 'Truth', swatch: 'var(--pr-truth)', gloss: 'ground truth' },
};

/** Legend chips placed under a figure or simulation so colors can be decoded. */
export function ColorKey({ items = PLANNING_ROLES }: ColorKeyProps) {
  return (
    <ul
      role="list"
      aria-label="Color key"
      className="not-prose my-3 flex flex-wrap items-center gap-x-4 gap-y-1.5"
    >
      {items.map((item) => {
        const role = ROLES[item];
        return (
          <li key={item} className="flex items-center gap-1.5" title={role.gloss}>
            <span
              aria-hidden="true"
              className="size-2.5 shrink-0 rounded-[1px]"
              style={{ backgroundColor: role.swatch }}
            />
            <span className="font-ui text-[0.6875rem] font-medium uppercase tracking-[0.1em] text-fd-muted-foreground">
              {role.label}
            </span>
          </li>
        );
      })}
    </ul>
  );
}
