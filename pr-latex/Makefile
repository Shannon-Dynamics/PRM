# Principles of Robot Motion via Rust — print edition
#
#   make          convert MDX → LaTeX and build the PDF
#   make convert  regenerate chapters/*.tex from the web edition only
#   make pdf      build the PDF from the current chapters/
#   make clean    remove build artefacts (keeps the PDF)
#   make dist     remove everything generated, including the PDF

TEXMF  := $(HOME)/.TinyTeX/bin/x86_64-linux
LATEX  := $(shell command -v pdflatex 2>/dev/null || echo $(TEXMF)/pdflatex)
PYTHON := python3
CHAPTERS := $(wildcard chapters/ch*.tex)

.PHONY: all convert pdf clean dist

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
	@echo "main.pdf  —  $$(pdfinfo main.pdf 2>/dev/null | awk '/Pages/{print $$2}') pages"

clean:
	@rm -f main.aux main.log main.out main.toc main.lof main.lot main.fls \
	       main.fdb_latexmk build/pass*.log
	@echo "cleaned build artefacts"

dist: clean
	@rm -f main.pdf
	@rm -f chapters/ch*.tex
	@echo "removed generated chapters and PDF"
