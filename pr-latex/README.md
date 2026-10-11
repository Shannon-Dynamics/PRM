# Principles of Robot Motion via Rust — print edition

The LaTeX edition of the book, generated from the interactive web edition in
`../web/` and typeset in the Shannon Robotics design language.

```sh
make          # convert MDX → LaTeX, then build main.pdf
make convert  # regenerate chapters/*.tex only
make pdf      # build the PDF from the current chapters/
make publish  # copy main.pdf to ../../Website/book-pdf/, where shannon.id serves it
make clean    # remove build artefacts, keep the PDF
make dist     # remove everything generated, including the PDF
```

Requires `pdflatex` with `newpx`, `tcolorbox`, `fontawesome5`, `listings` and
`microtype` — a full TeX Live covers all of it. On this machine it is TinyTeX at
`~/.TinyTeX` (the Makefile finds it there), plus the packages TinyTeX leaves out:

```sh
tlmgr install newpx newtx txfonts kastrup fontaxes xstring xkeyval tcolorbox pgf \
  environ trimspaces fontawesome5 listings listingsutf8 microtype titlesec enumitem \
  fancyhdr caption booktabs colortbl mathtools helvetic bera tools etoolbox pdfcol \
  tikzfill psnfss iftex xpatch carlisle oberdiek tex-gyre
```

A clean build reports no LaTeX errors, no undefined references, and no overfull
box wider than about a point (`grep '^Overfull' build/pass3.log`); keep it so.

## How it works

`build/mdx2tex.py` is the whole pipeline. The web edition stays the single source
of truth for content; the script projects it onto the print design system:

| MDX | Print |
|---|---|
| frontmatter `quote` / `quoteAuthor` / `quoteSource` | chapter epigraph block |
| `<Overview goals prerequisites>` | opening prose, then "after this chapter you can" and "assumed" boxes |
| `<Derivation title result>` | a marked derivation block with the result stated on its rule |
| `<Algorithm name inputs outputs complexity>` | Choset-style numbered algorithm box |
| `<Exercise level difficulty>` | exercise card with an F/C/P badge and difficulty dots |
| `<Reference authors year title venue doi url note>` | hanging-indent bibliography entry |
| `<NotationTable rows>` | two-column symbol table |
| `<KeyIdea>` / `<Callout>` | insight and note callouts |
| ` ```rust ` fences | captioned `shrust` listing with line numbers |
| `$…$`, `$$…$$` | passed through — it is already LaTeX |
| `\htmlClass{term-path}{…}` | `\pathterm{…}`, so equation tinting survives into ink |
| a widget component | a described figure carrying the widget's id and teaching point |

**Widget descriptions are read out of the React source**, not maintained
separately: the script scans `../web/components/ch/*/*.tsx` for each
`WidgetFrame`'s `id`, `title` and `teaches`. A simulation cannot exist on paper,
so the print edition says what it would show you and where to run it — and it
cannot drift from the interactive edition, because it is generated from it.

## Design

`shannon.sty` carries the design language, shared with the companion volume
*Reinforcement Learning for Robotics*: Space Grotesk display type (Helvetica
substitutes under pdfLaTeX), a Palatino-class serif for long-form reading, the
slate neutral scale, and teal as the single chrome accent. Flat card-like blocks
with a coloured leading rule; no gradients, no shadows.

This volume adds what a book about motion planning needs:

- **The planning palette** — `\startterm`, `\goalterm`, `\obstacleterm`,
  `\cobstacleterm`, `\graphterm`, `\pathterm`, `\robotterm` (TOC.md §2),
  deepened from the screen values for ink. Green is always the start, red the
  goal, amber the C-obstacle, blue what a planner explored, purple the path it
  chose, orange the robot — in the prose, the equations and the figures alike.
  `\shplanningkey` prints the legend. The sister volume's estimation palette
  (`\prior` … `\truthterm`, `\shcolorkey`) is kept for the belief chapters.
- **The book's math macros** — `\Q`, `\QO`, `\Qfree`, `\qstart`, `\lie`,
  `\SEtwo`, `\bel` and the rest, mirrored from `../web/lib/katex-macros.ts` so
  both editions render identical mathematics from identical source. Add a macro
  there and here together.
- **`\shfitdisplay`** — the web edition scrolls a wide equation sideways; paper
  cannot, so a one-line display is measured and scaled to the measure only if it
  would overrun it.
- **`shderivation`**, the print form of the web edition's collapsible algebra.
- **`shalgorithm`**, numbered pseudocode in the style of Choset's algorithm boxes.
- **`shnotation`**, the per-chapter symbol table.
- **Widget cards** print the full address of each simulation in the web edition
  (`\shwebedition`, set to https://prm.shannon.id; change it there if the book
  moves to its own domain).

## Editing

Don't edit `chapters/*.tex` — they are generated and `make convert` overwrites
them. Fix the MDX in `../web/content/chapters/`, or the converter, or
`shannon.sty`.
