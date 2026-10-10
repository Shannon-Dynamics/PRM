# Principles of Robot Motion via Rust — print edition

The LaTeX edition of the book, generated from the interactive web edition in
`../web/` and typeset in the Shannon Robotics design language.

```sh
make          # convert MDX → LaTeX, then build main.pdf
make convert  # regenerate chapters/*.tex only
make pdf      # build the PDF from the current chapters/
make clean    # remove build artefacts, keep the PDF
make dist     # remove everything generated, including the PDF
```

Requires `pdflatex` with `newpx`, `tcolorbox`, `fontawesome5`, `listings` and
`microtype` — a full TeX Live or TinyTeX install covers all of it.

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
| `\htmlClass{term-prior}{…}` | `\prior{…}`, so equation tinting survives into ink |
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

This volume adds what a book about estimation needs:

- **The estimation palette** — `\prior`, `\prediction`, `\measurement`,
  `\posterior`, `\truthterm`, deepened from the screen values for ink. These are
  reserved for data and never used for chrome, so blue always means prior and
  purple always means posterior, in the prose, in the equations and in the
  figures alike. `\shcolorkey` prints the legend.
- **The book's math macros** — `\bel`, `\belbar`, `\SEtwo`, `\bplus`, `\Normal`
  and the rest, ported from `../web/lib/katex-macros.ts` so both editions render
  identical mathematics from identical source.
- **`shderivation`**, the print form of the web edition's collapsible algebra.
- **`shalgorithm`**, numbered pseudocode in the style of Choset's algorithm boxes.
- **`shnotation`**, the per-chapter symbol table.

## Editing

Don't edit `chapters/*.tex` — they are generated and `make convert` overwrites
them. Fix the MDX in `../web/content/chapters/`, or the converter, or
`shannon.sty`.
