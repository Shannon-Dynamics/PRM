import type { Metadata } from 'next';
import { NotationTable } from '@/components/book/notation-table';
import { ColorKey } from '@/components/book/color-key';
import { SiteHeader } from '@/components/shannon/site-header';
import { PageBanner, bookCrumb } from '@/components/shannon/page-banner';
import { BOOK_LINKS } from '@/lib/nav';

export const metadata: Metadata = {
  title: 'Notation',
  description:
    'Every symbol used in Principles of Robot Motion via Rust, following Choset et al., extended where the modern planning literature demands it.',
  alternates: { canonical: '/notation/' },
};

export default function NotationPage() {
  return (
    <>
      <SiteHeader bookLinks={BOOK_LINKS} overlay />
      <PageBanner
        small
        crumb={bookCrumb(
          { label: 'PRINCIPLES OF ROBOT MOTION VIA RUST', href: '/' },
          { label: 'NOTATION' },
        )}
        title="Notation"
        sub="Every symbol used in the book, following Choset et al., extended where the modern planning literature demands it."
      />

      <main className="mx-auto w-full max-w-3xl px-4 py-14 sm:px-6">
        <p className="font-prose text-base leading-relaxed text-fd-muted-foreground">
          The book follows the notation of Choset, Lynch, Hutchinson, Kantor, Burgard, Kavraki and
          Thrun so that readers can move between this text and the literature without translating.
          Where the field has moved on — asymptotically optimal planners, trajectory optimization —
          the notation is extended rather than replaced. Each chapter adds only the symbols it
          introduces, in its own notation table.
        </p>

        <section className="mt-10">
          <h2 className="font-display text-xl font-semibold">Workspace and configuration space</h2>
          <NotationTable
            rows={[
              { sym: '\\W \\subset \\R^2 \\text{ or } \\R^3', meaning: 'The workspace: the ambient space the robot and obstacles live in' },
              { sym: '\\WO_i,\; \\Wfree', meaning: 'The i-th workspace obstacle; the free workspace' },
              { sym: '\\Q,\; q', meaning: 'The configuration space and one configuration; dim Q is the number of degrees of freedom' },
              { sym: '\\Rq \\subset \\W', meaning: 'The set of workspace points occupied by the robot at configuration q' },
              {
                sym: '\\QO_i = \\{ q : \\Rq \\cap \\WO_i \\neq \\emptyset \\}',
                meaning: 'The C-obstacle: every configuration in which the robot touches obstacle i',
              },
              { sym: '\\Qfree = \\Q \\setminus \\bigcup_i \\QO_i', meaning: 'Free configuration space' },
              {
                sym: 'c : [0,1] \\to \\Qfree',
                meaning: 'A path — a continuous curve in free space; a trajectory when the parameter is time',
                note: 'Path planning finds c; trajectory planning finds c(t) with velocities and accelerations',
              },
              { sym: '\\qstart,\; \\qgoal', meaning: 'The query endpoints' },
              { sym: 'd(x, y),\; d_i(q),\; D(x)', meaning: 'A metric; the distance from q to C-obstacle i; the distance from a workspace point to the nearest obstacle' },
              { sym: '\\rho(x, s),\; \\rho_R(x, s)', meaning: 'Raw range along the ray at angle s from x; the same saturated at the sensor range R' },
            ]}
          />
        </section>

        <section className="mt-10">
          <h2 className="font-display text-xl font-semibold">Topology and kinematics</h2>
          <NotationTable
            rows={[
              { sym: 'S^1,\; T^n,\; \\SOtwo,\; \\SEtwo,\; \\SOthree,\; \\SEthree', meaning: 'The circle, the n-torus, and the rotation and rigid-motion groups of the plane and of space' },
              { sym: '\\varphi : \\Q \\to \\R^n,\; D\\varphi,\; J(q)', meaning: 'The forward kinematic map, its differential, the Jacobian' },
              { sym: '\\Tq', meaning: 'The tangent space at q: the velocities available there' },
              { sym: 'f(q),\; g_i(q)', meaning: 'Drift vector field and control vector fields of a control system' },
              { sym: '\\lie{f}{g},\; \\Dist,\; \\overline{\\Dist}', meaning: 'The Lie bracket; a distribution; its involutive closure' },
              { sym: '\\omega(q)\\,\\dot q = 0', meaning: 'A Pfaffian (velocity) constraint' },
            ]}
          />
        </section>

        <section className="mt-10">
          <h2 className="font-display text-xl font-semibold">Potentials, graphs, and samplers</h2>
          <NotationTable
            rows={[
              { sym: 'U(q) = U_{att}(q) + U_{rep}(q),\; \\nabla U', meaning: 'An artificial potential and its gradient' },
              { sym: 'G = (V, E),\; g(v),\; h(v),\; f(v) = g(v) + h(v)', meaning: 'A graph; cost-to-come; heuristic; the A* priority' },
              { sym: 'n,\; k,\; r_n', meaning: 'Sample count, neighbor count, and the connection radius of a sampling-based planner' },
              { sym: '(\\epsilon, \\alpha, \\beta)', meaning: 'The expansiveness parameters of free space, which govern how fast a roadmap captures its connectivity' },
            ]}
          />
        </section>

        <section className="mt-10">
          <h2 className="font-display text-xl font-semibold">Dynamics and trajectories</h2>
          <NotationTable
            rows={[
              { sym: 'M(q)\\ddot q + C(q, \\dot q)\\dot q + g(q) = \\tau', meaning: 'The standard form of a robot\'s equations of motion: inertia, Coriolis/centrifugal, gravity, joint torques' },
              { sym: 's,\; \\dot s', meaning: 'Path parameter and its rate — the phase plane of time-optimal trajectory planning' },
              { sym: '\\tau_{\\min},\; \\tau_{\\max}', meaning: 'Actuator torque limits' },
            ]}
          />
        </section>

        <section className="mt-10">
          <h2 className="font-display text-xl font-semibold">Where estimation appears</h2>
          <p className="mt-2 font-prose text-sm leading-relaxed text-fd-muted-foreground">
            Chapters 15–16 and the capstone use the sister book\'s vocabulary unchanged: the state{' '}
            <code>x_t</code>, control <code>u_t</code>, measurement <code>z_t</code>, and belief{' '}
            <code>bel(x_t)</code>. For Rusty the planner\'s <code>q</code> and the filter\'s{' '}
            <code>x_t</code> are the same numbers; the text says so once, where they meet.
          </p>
        </section>

        <section className="mt-10">
          <h2 className="font-display text-xl font-semibold">The color code</h2>
          <p className="mt-2 font-prose text-sm leading-relaxed text-fd-muted-foreground">
            Figures, color-coded equation terms, widget controls, and code comments all use the same
            seven planning roles. Where a belief is drawn, the sister book\'s five estimation colors
            are imported unchanged.
          </p>
          <div className="mt-3">
            <ColorKey />
          </div>
          <div className="mt-1">
            <ColorKey items={['prior', 'prediction', 'measurement', 'posterior', 'truth']} />
          </div>
        </section>
      </main>
    </>
  );
}
