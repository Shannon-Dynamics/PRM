# Principles of Robot Motion via Rust — print edition
#
#   make          convert MDX → LaTeX and build the PDF
#   make convert  regenerate chapters/*.tex from the web edition only
#   make pdf      build the PDF from the current chapters/
#   make publish  copy the PDF to the marketing site, which serves it
#   make clean    remove build artefacts (keeps the PDF)
#   make dist     remove everything generated, including the PDF

# TinyTeX installs to ~/.TinyTeX on Linux and ~/Library/TinyTeX on macOS.
TEXMF  := $(firstword $(wildcard $(HOME)/.TinyTeX/bin/x86_64-linux $(HOME)/Library/TinyTeX/bin/universal-darwin))
LATEX  := $(shell command -v pdflatex 2>/dev/null || echo $(TEXMF)/pdflatex)
PYTHON := python3
CHAPTERS := $(wildcard chapters/ch*.tex)
# The site publishes every PDF edition from book-pdf/; the web edition's
# "Download PDF" button (web/lib/shannon.ts BOOK_PDF) links to it there.
SITE_PDF := ../../Website/book-pdf/principles-of-robot-motion-via-rust.pdf

.PHONY: all convert pdf publish clean dist

all: convert pdf

convert:
	@$(PYTHON) build/mdx2tex.py

# Three passes: contents, cross-references, then final page numbers.
pdf: main.tex shannon.sty $(CHAPTERS)
	@$(LATEX) -interaction=nonstopmode main.tex > build/pass1.log 2>&1 || true
	@$(LATEX) -interaction=nonstopmode main.tex > build/pass2.log 2>&1 || true
	@$(LATEX) -interaction=nonstopmode main.tex > build/pass3.log 2>&1 || true
	@errors=$$(grep -acE '^!' build/pass3.log || true); \
	 if [ "$$errors" != "0" ]; then \
	   echo "LaTeX reported $$errors error(s):"; grep -aE '^!' -A3 build/pass3.log | head -40; exit 1; \
	 fi
	@echo "main.pdf  —  $$(LC_ALL=C sed -n 's/^Output written on main.pdf (\([0-9]*\) pages.*/\1/p' build/pass3.log) pages"

publish: main.pdf
	@test -d $(dir $(SITE_PDF)) || { echo "no site checkout at $(dir $(SITE_PDF))"; exit 1; }
	@cp main.pdf $(SITE_PDF)
	@echo "published → $(SITE_PDF)"

clean:
	@rm -f main.aux main.log main.out main.toc main.lof main.lot main.fls \
	       main.fdb_latexmk build/pass*.log
	@echo "cleaned build artefacts"

dist: clean
	@rm -f main.pdf
	@rm -f chapters/ch*.tex
	@echo "removed generated chapters and PDF"
