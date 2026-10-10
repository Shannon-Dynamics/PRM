import Link from 'next/link';
import { existsSync } from 'node:fs';
import { join } from 'node:path';
import { HeroPlanner } from '@/components/home/hero-planner';
import { PARTS } from '@/lib/book-structure';
import { PageBanner, bookCrumb } from '@/components/shannon/page-banner';

/** Build-time check: which chapters have been written so far. */
const isWritten = (slug: string) =>
  existsSync(join(process.cwd(), 'content', 'chapters', `${slug}.mdx`));

export default function HomePage() {
  return (
    <>
      <PageBanner
        crumb={bookCrumb({ label: 'PRINCIPLES OF ROBOT MOTION VIA RUST' })}
        title="Principles of Robot Motion via Rust"
        sub="Rusty the rover, Reach the planar arm, and Hitch the car with a trailer — from a bug crawling around an obstacle to a complete planning stack."
      />

      <main className="mx-auto w-full max-w-6xl px-4 py-14 sm:px-6 lg:py-20">
        {/* ---------------------------------------------------------------- */}
        {/* Hero: the thesis, stated and then shown running.                  */}
        {/* ---------------------------------------------------------------- */}
        <section className="grid items-center gap-10 lg:grid-cols-[1.05fr_1fr] lg:gap-14">
          <div>
            <p className="sd-kicker">An interactive book · Foundation · Conceptual · Practical</p>
            <h2 className="mt-4 font-display text-3xl leading-[1.12] font-semibold tracking-tight sm:text-4xl">
              A robot that knows where it is still has to decide how to move.
            </h2>
            <p className="mt-5 max-w-xl font-prose text-lg leading-relaxed text-fd-foreground/85">
              This book takes that problem seriously: it derives the geometry and the algorithms of motion planning in
              full, makes every hard idea something you can <em>play with</em> in the page, and
              implements all of it in Rust.
            </p>
            <p className="mt-4 max-w-xl font-prose text-base leading-relaxed text-fd-muted-foreground">
              Twenty-three chapters take three robots — Rusty the rover, Reach the planar arm, and
              Hitch the car with a trailer — from a bug crawling around an obstacle to a complete
              planning stack: configuration space, roadmaps, sampling-based and optimal planners,
              dynamics and time-optimal trajectories, and the differential geometry of systems that
              cannot move sideways.
            </p>

            <div className="mt-8 flex flex-wrap items-center gap-3">
              <Link href="/chapters" className="sd-btn pri">
                Start reading <span aria-hidden="true">→</span>
              </Link>
              <Link href="/chapters/ch04-configuration-space-i" className="sd-btn">
                Jump to configuration space
              </Link>
            </div>
          </div>

          <div>
            <HeroPlanner />
            <p className="mt-2 font-ui text-xs leading-relaxed text-fd-muted-foreground">
              <strong className="font-semibold text-fd-foreground">A rapidly-exploring random tree,
              growing live.</strong>{' '}
              Every step samples a random configuration, finds the nearest node of the tree, and
              extends toward it — so the tree is pulled into the largest unexplored regions first.
              Within seconds it threads the apartment and finds the goal. That bias is what
              Chapter 12 explains; Chapter 13 shows what it costs in path quality, and how to fix it.
            </p>
          </div>
        </section>

        {/* ---------------------------------------------------------------- */}
        {/* The method                                                        */}
        {/* ---------------------------------------------------------------- */}
        <section className="mt-20 border-t border-fd-border pt-10">
          <p className="sd-kicker sd-kicker-accent">The method</p>
          <h2 className="mt-2 font-display text-2xl font-semibold tracking-tight">Three passes over every idea</h2>
          <div className="mt-6 grid gap-px overflow-hidden rounded-xl border border-fd-border bg-fd-border sm:grid-cols-3">
            {[
              {
                k: 'F',
                t: 'Foundation',
                d: 'The full mathematics: configuration spaces as manifolds, completeness and optimality stated precisely, proofs sketched in the text and carried through in collapsible blocks.',
              },
              {
                k: 'C',
                t: 'Conceptual',
                d: 'Every hard idea becomes something you can manipulate. Drag an obstacle and watch a C-obstacle deform, grow a tree, scrub a phase-plane trajectory, parallel-park with a Lie bracket.',
              },
              {
                k: 'P',
                t: 'Practical',
                d: 'Then you build it in Rust, with the crates the field actually uses — nalgebra, parry, petgraph — and code you could lift into a real planner.',
              },
            ].map((c) => (
              <div key={c.k} className="bg-fd-card p-5">
                <div className="flex items-baseline gap-2">
                  <span className="font-mono text-sm font-semibold text-fd-primary">{c.k}</span>
                  <h3 className="font-display text-base font-semibold">{c.t}</h3>
                </div>
                <p className="mt-2 font-prose text-sm leading-relaxed text-fd-muted-foreground">
                  {c.d}
                </p>
              </div>
            ))}
          </div>
        </section>

        {/* ---------------------------------------------------------------- */}
        {/* Contents                                                          */}
        {/* ---------------------------------------------------------------- */}
        <section className="mt-16">
          <p className="sd-kicker sd-kicker-accent">Contents</p>
          <h2 className="mt-2 font-display text-2xl font-semibold tracking-tight">Six parts, twenty-three chapters</h2>
          <div className="mt-6 space-y-8">
            {PARTS.map((part) => (
              <div key={part.id}>
                <div className="flex items-baseline gap-3 border-b border-fd-border pb-1.5">
                  <span className="sd-kicker sd-kicker-accent">{part.id}</span>
                  <h3 className="font-display text-base font-semibold">{part.title}</h3>
                </div>
                <ul className="mt-2">
                  {part.chapters.map((ch) => {
                    const written = isWritten(ch.slug);
                    const inner = (
                      <>
                        <span className="pt-0.5 font-mono text-[0.7rem] tracking-widest tabular-nums text-fd-muted-foreground">
                          CH.{String(ch.n).padStart(2, '0')}
                        </span>
                        <span>
                          <span className={`font-ui text-sm font-medium ${written ? 'group-hover:text-fd-primary' : 'text-fd-muted-foreground'}`}>
                            {ch.title}
                            {written ? null : (
                              <span className="ms-2 align-middle font-ui text-[0.65rem] font-medium uppercase tracking-[0.12em] text-fd-muted-foreground/70">
                                in preparation
                              </span>
                            )}
                          </span>
                          <span className="block font-prose text-[0.82rem] leading-snug text-fd-muted-foreground">
                            {ch.blurb}
                          </span>
                        </span>
                      </>
                    );
                    return (
                      <li key={ch.n} className="border-b border-fd-border/60 last:border-0">
                        {written ? (
                          <Link
                            href={`/chapters/${ch.slug}`}
                            className="group grid grid-cols-[2.5rem_1fr] gap-3 rounded-lg py-2.5 transition-[background-color,padding] hover:bg-fd-accent hover:ps-3"
                          >
                            {inner}
                          </Link>
                        ) : (
                          <div className="grid grid-cols-[2.5rem_1fr] gap-3 py-2.5 opacity-80">{inner}</div>
                        )}
                      </li>
                    );
                  })}
                </ul>
              </div>
            ))}
          </div>
        </section>

        {/* ---------------------------------------------------------------- */}
        {/* Sister volume                                                     */}
        {/* ---------------------------------------------------------------- */}
        <section className="mt-16 border-t border-fd-border pt-10">
          <p className="sd-kicker sd-kicker-accent">Sister volume</p>
          <h2 className="mt-2 font-display text-2xl font-semibold tracking-tight">Where am I? is the other book.</h2>
          <p className="mt-3 max-w-2xl font-prose text-base leading-relaxed text-fd-muted-foreground">
            This book shares its robot, its simulator, and its method with{' '}
            <a
              href="https://shannon-dynamics.github.io/ProbabilisticRoboticRust/"
              className="text-fd-primary underline-offset-2 hover:underline"
            >
              Probabilistic Robotics via Rust
            </a>
            , which treats estimation — Bayes filters, Kalman and particle filters, SLAM — in
            twenty-six chapters. Part IV here covers exactly as much of that as a planner needs, and
            links there for the rest.
          </p>
        </section>
      </main>
    </>
  );
}
