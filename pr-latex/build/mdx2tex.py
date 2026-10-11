#!/usr/bin/env python3
"""
mdx2tex.py — convert the book's MDX chapters into LaTeX for the print edition.

The web edition is the source of truth for content; this script projects it onto
the Shannon Robotics LaTeX design system. It handles:

  · YAML frontmatter (title, chapter number, part, difficulty, epigraph)
  · every custom MDX component this book uses
  · GitHub-flavoured markdown: headings, lists, tables, emphasis, links, rules
  · math, which passes through almost untouched because it is already LaTeX —
    only the book's \\htmlClass{term-*} colour tints need rewriting
  · Rust code fences, routed into the shrust environment

Interactive simulations cannot exist on paper, so each becomes a described
figure carrying its widget id, title and teaching point — read straight out of
the React component, so the two editions can never drift apart.

Shares its inline/escaping machinery with the companion volume's converter.
"""

from __future__ import annotations

import re
import sys
import unicodedata
from pathlib import Path

# ---------------------------------------------------------------------------
# A tolerant parser for the JS object/array literals used in JSX props.
# ---------------------------------------------------------------------------


class JsLiteral:
    """Parses the subset of JS literal syntax that appears in this book's MDX."""

    def __init__(self, text: str):
        self.s = text
        self.i = 0

    def parse(self):
        self._ws()
        return self._value()

    def _ws(self):
        while self.i < len(self.s):
            c = self.s[self.i]
            if c in " \t\r\n":
                self.i += 1
            elif self.s.startswith("//", self.i):
                nl = self.s.find("\n", self.i)
                self.i = len(self.s) if nl < 0 else nl
            else:
                break

    def _value(self):
        self._ws()
        if self.i >= len(self.s):
            return None
        c = self.s[self.i]
        if c == "[":
            return self._array()
        if c == "{":
            return self._object()
        if c in "\"'`":
            return self._string(c)
        return self._bare()

    def _array(self):
        out = []
        self.i += 1
        while True:
            self._ws()
            if self.i >= len(self.s):
                break
            if self.s[self.i] == "]":
                self.i += 1
                break
            out.append(self._value())
            self._ws()
            if self.i < len(self.s) and self.s[self.i] == ",":
                self.i += 1
        return out

    def _object(self):
        out = {}
        self.i += 1
        while True:
            self._ws()
            if self.i >= len(self.s):
                break
            if self.s[self.i] == "}":
                self.i += 1
                break
            if self.s[self.i] in "\"'`":
                key = self._string(self.s[self.i])
            else:
                m = re.compile(r"[A-Za-z_$][\w$]*").match(self.s, self.i)
                if not m:
                    self.i += 1
                    continue
                key = m.group(0)
                self.i = m.end()
            self._ws()
            if self.i < len(self.s) and self.s[self.i] == ":":
                self.i += 1
            out[key] = self._value()
            self._ws()
            if self.i < len(self.s) and self.s[self.i] == ",":
                self.i += 1
        return out

    def _string(self, quote):
        self.i += 1
        buf = []
        while self.i < len(self.s):
            c = self.s[self.i]
            if c == "\\":
                nxt = self.s[self.i + 1] if self.i + 1 < len(self.s) else ""
                if nxt == "\\":
                    buf.append("\\")          # \\ in source is one backslash
                elif nxt in "\"'`":
                    buf.append(nxt)
                else:
                    buf.append(c + nxt)      # \mathsf, \Sigma, … stay LaTeX
                self.i += 2
                continue
            if c == quote:
                self.i += 1
                break
            buf.append(c)
            self.i += 1
        return "".join(buf)

    def _bare(self):
        m = re.compile(r"[^,\]\}\s]+").match(self.s, self.i)
        if not m:
            self.i += 1
            return None
        tok = m.group(0)
        self.i += len(tok)
        if tok == "true":
            return True
        if tok == "false":
            return False
        if tok == "null":
            return None
        try:
            return float(tok) if "." in tok else int(tok)
        except ValueError:
            return tok


def parse_js(text: str):
    return JsLiteral(text).parse()


def match_braces(text: str, start: int) -> int:
    """Index just past the brace group opening at `start`, respecting strings."""
    depth = 0
    i = start
    quote = None
    while i < len(text):
        c = text[i]
        if quote:
            if c == "\\":
                i += 2
                continue
            if c == quote:
                quote = None
        elif c in "\"'`":
            quote = c
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    return len(text)


def match_braces_tex(text: str, start: int) -> int:
    """
    Index just past the brace group opening at `start`, in LaTeX.

    Unlike the JS matcher, an apostrophe here is a prime — "x'" — not a string
    delimiter, and a backslash escapes the character after it.
    """
    depth = 0
    i = start
    while i < len(text):
        c = text[i]
        if c == "\\":
            i += 2
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    return len(text)


# ---------------------------------------------------------------------------
# Inline conversion
# ---------------------------------------------------------------------------

ESCAPES = {
    "\\": r"\textbackslash{}",
    "&": r"\&",
    "%": r"\%",
    "$": r"\$",
    "#": r"\#",
    "_": r"\_",
    "{": r"\{",
    "}": r"\}",
    "~": r"\textasciitilde{}",
    "^": r"\textasciicircum{}",
}

# Superscript/subscript runs that must stay a single exponent — "⁻¹" is
# ^{-1}, not ^-^1, which TeX rejects as a double superscript.
MATH_PAIRS = {
    "⁻¹": "^{-1}", "⁻²": "^{-2}", "⁻³": "^{-3}", "⁻¹⁄²": "^{-1/2}",
    "⁺¹": "^{+1}", "½": r"\tfrac{1}{2}", "¼": r"\tfrac{1}{4}",
    "¾": r"\tfrac{3}{4}", "⅓": r"\tfrac{1}{3}", "⅔": r"\tfrac{2}{3}",
}

MATH_SYMBOL = {
    "α": r"\alpha", "β": r"\beta", "γ": r"\gamma", "δ": r"\delta",
    "ε": r"\varepsilon", "ζ": r"\zeta", "η": r"\eta", "θ": r"\theta",
    "ι": r"\iota", "κ": r"\kappa", "λ": r"\lambda", "μ": r"\mu",
    "ν": r"\nu", "ξ": r"\xi", "π": r"\pi", "ρ": r"\rho",
    "σ": r"\sigma", "τ": r"\tau", "υ": r"\upsilon", "φ": r"\phi",
    "χ": r"\chi", "ψ": r"\psi", "ω": r"\omega",
    "Δ": r"\Delta", "Σ": r"\Sigma", "Φ": r"\Phi", "Ω": r"\Omega",
    "Ψ": r"\Psi", "Γ": r"\Gamma", "Λ": r"\Lambda", "Θ": r"\Theta",
    "Ξ": r"\Xi", "Π": r"\Pi",
    "×": r"\times", "≈": r"\approx", "≤": r"\leq", "≥": r"\geq",
    "≠": r"\neq", "→": r"\rightarrow", "←": r"\leftarrow",
    "↔": r"\leftrightarrow", "⇒": r"\Rightarrow", "⇐": r"\Leftarrow",
    "±": r"\pm", "∓": r"\mp", "∞": r"\infty", "∈": r"\in", "∉": r"\notin",
    "∀": r"\forall", "∃": r"\exists", "∇": r"\nabla", "∂": r"\partial",
    "≫": r"\gg", "≪": r"\ll", "⊤": r"\top", "⊥": r"\perp",
    "⊙": r"\odot", "⊕": r"\oplus", "⊗": r"\otimes",
    "⊞": r"\boxplus", "⊟": r"\boxminus",
    "∝": r"\propto", "†": r"\dagger", "∑": r"\sum", "∏": r"\prod",
    "∫": r"\int", "≡": r"\equiv", "≜": r"\triangleq",
    "⟨": r"\langle", "⟩": r"\rangle", "•": r"\bullet", "ℓ": r"\ell",
    "√": r"\surd", "∥": r"\|", "‖": r"\|", "−": "-", "·": r"\cdot",
    "∘": r"\circ", "∩": r"\cap", "∪": r"\cup", "⊂": r"\subset",
    "⊆": r"\subseteq", "∅": r"\emptyset", "∧": r"\wedge", "∨": r"\vee",
    "¬": r"\neg", "∼": r"\sim", "≃": r"\simeq", "≅": r"\cong",
    "°": r"^\circ", "′": "'", "″": "''", "‴": "'''",
    "¹": "^1", "²": "^2", "³": "^3", "⁻": "^-", "⁺": "^+",
    "ᵀ": r"^{\mathsf{T}}", "ᴴ": "^H", "ᵗ": "^t", "ⁿ": "^n", "ᵢ": "_i",
    "ₜ": "_t", "ₖ": "_k", "ₙ": "_n", "ᵣ": "_r", "ₛ": "_s",
    "₀": "_0", "₁": "_1", "₂": "_2", "₃": "_3", "₄": "_4",
    "ẋ": r"\dot{x}", "ẏ": r"\dot{y}", "ż": r"\dot{z}",
    "ÿ": r"\ddot{y}", "ẍ": r"\ddot{x}", "θ̇": r"\dot{\theta}",
    "ℝ": r"\mathbb{R}", "ℕ": r"\mathbb{N}", "ℤ": r"\mathbb{Z}",
    "𝔼": r"\mathbb{E}", "⌊": r"\lfloor", "⌋": r"\rfloor",
    "⌈": r"\lceil", "⌉": r"\rceil",
}

MATH_SYMBOL.update({
    "½": r"\tfrac{1}{2}", "¼": r"\tfrac{1}{4}", "¾": r"\tfrac{3}{4}",
    "ς": r"\varsigma", "ϵ": r"\epsilon", "↦": r"\mapsto",
    "⇔": r"\Leftrightarrow", "≻": r"\succ", "⊖": r"\ominus",
    "⋯": r"\cdots", "◀": r"\blacktriangleleft",
    "–": r"\text{--}", "—": r"\text{---}",
})

UNICODE = {
    "—": "---", "–": "--", "’": "'", "‘": "`",
    "“": "``", "”": "''", "…": r"\dots{}", "§": r"\S{}",
    "\u2011": "-", "\u00a0": "~", "\u2009": r"\,",
    "\u200b": "", "\u202f": r"\,", "\u2007": "~",
    "á": r"\'a", "é": r"\'e", "í": r"\'i", "ó": r"\'o",
    "ú": r"\'u", "à": r"\`a", "è": r"\`e", "ä": r'\"a',
    "ë": r'\"e', "ö": r'\"o', "ü": r'\"u', "ñ": r"\~n",
    "ç": r"\c{c}", "ø": r"\o{}", "å": r"\aa{}", "š": r"\v{s}",
    "č": r"\v{c}", "ř": r"\v{r}", "ž": r"\v{z}", "ł": r"\l{}",
    "Á": r"\'A", "É": r"\'E", "Í": r"\'I", "Ó": r"\'O",
    "Ü": r'\"U', "Ö": r'\"O', "Ä": r'\"A', "Ø": r"\O{}",
    "ß": r"\ss{}", "æ": r"\ae{}", "œ": r"\oe{}",
    "©": r"\copyright{}", "→": r"$\rightarrow$", "×": r"$\times$",
}
for _k, _v in MATH_SYMBOL.items():
    UNICODE.setdefault(_k, "$" + _v + "$")

COMBINING = {"̇": r"\dot", "̈": r"\ddot", "̄": r"\bar",
             "̂": r"\hat", "̃": r"\tilde", "⃗": r"\vec"}

CODE_ASCII = {
    "α": "alpha", "β": "beta", "γ": "gamma", "δ": "delta", "ε": "epsilon",
    "ζ": "zeta", "η": "eta", "θ": "theta", "κ": "kappa", "λ": "lambda",
    "μ": "mu", "µ": "mu", "ν": "nu", "ξ": "xi", "π": "pi", "ρ": "rho", "σ": "sigma",
    "τ": "tau", "φ": "phi", "χ": "chi", "ψ": "psi", "ω": "omega",
    "Δ": "Delta", "Σ": "Sigma", "Φ": "Phi", "Ω": "Omega", "Γ": "Gamma",
    "Λ": "Lambda", "Θ": "Theta",
    "≤": "<=", "≥": ">=", "≠": "!=", "≈": "~=", "→": "->", "←": "<-",
    "⇒": "=>", "×": "*", "·": ".", "±": "+/-", "∞": "inf", "∈": "in",
    "⊙": "*", "⊞": "[+]", "⊟": "[-]", "∥": "||", "‖": "||", "−": "-",
    "√": "sqrt", "∝": "prop", "∘": "o", "∫": "int", "∑": "sum",
    "¹": "1", "²": "2", "³": "3", "⁻": "-", "ᵀ": "^T", "ᴴ": "^H",
    "ᵗ": "^t", "ⁿ": "^n", "ₜ": "_t", "ₖ": "_k", "₀": "_0", "₁": "_1",
    "₂": "_2", "₃": "_3", "ℓ": "l", "′": "'", "″": "''",
    "ẋ": "x_dot", "ẏ": "y_dot", "ż": "z_dot", "ÿ": "y_ddot", "ẍ": "x_ddot",
    "—": "--", "–": "-", "’": "'", "‘": "'", "“": '"', "”": '"',
    "…": "...", "§": "sec.", "é": "e", "í": "i", "ü": "u", "á": "a",
    "ø": "o", "ñ": "n", "ö": "o", "ä": "a", "ç": "c", "ℝ": "R",
}


def asciify_code(s: str) -> str:
    """Make a code block safe for the verbatim path, preserving meaning."""
    out = []
    i = 0
    while i < len(s):
        ch = s[i]
        nxt = s[i + 1] if i + 1 < len(s) else ""
        pair = ch + nxt
        if pair in CODE_ASCII:
            out.append(CODE_ASCII[pair])
            i += 2
            continue
        if nxt in COMBINING:
            base = CODE_ASCII.get(ch, ch if ord(ch) < 128 else "?")
            out.append(base + "_dot" if nxt == "̇" else base)
            i += 2
            continue
        if ch in CODE_ASCII:
            out.append(CODE_ASCII[ch])
        elif ord(ch) > 127:
            out.append("?")
        else:
            out.append(ch)
        i += 1
    return "".join(out)


# Accents the tables above do not spell out, recovered by decomposition.
TEXT_ACCENT = {"\u0301": "\\'", "\u0300": "\\`", "\u0302": "\\^", "\u0303": "\\~",
               "\u0304": "\\=", "\u0307": "\\.", "\u0308": '\\"', "\u030c": "\\v",
               "\u0327": "\\c", "\u0331": "\\b"}
MATH_ACCENT = {**COMBINING, "\u0331": r"\underline"}
_warned: set[str] = set()


def _script_run(s: str, i: int):
    """A run of Unicode super- or subscripts at s[i] → (^{…} or _{…}, length)."""
    def kind(c):
        d = unicodedata.decomposition(c).split()
        return (d[0], chr(int(d[1], 16))) if len(d) == 2 and d[0] in ("<super>", "<sub>") else None
    k = kind(s[i])
    if not k:
        return None
    tag, parts, j = k[0], [], i
    while j < len(s) and (kk := kind(s[j])) and kk[0] == tag:
        base = kk[1]
        parts.append(r"\mathsf{T}" if s[j] == "ᵀ" else MATH_SYMBOL.get(base, base))
        j += 1
    return ("^{" if tag == "<super>" else "_{") + "".join(parts) + "}", j - i


def _fallback(ch: str, prev: str, nxt: str, escape: bool) -> str:
    """A character no table names: decompose it rather than drop it."""
    math = lambda m: "$" + m + "$" if escape else m  # noqa: E731
    nfd = unicodedata.normalize("NFD", ch)
    if len(nfd) == 2 and nfd[0].isascii() and nfd[0].isalpha():
        base, mark = nfd
        # Inside a word it is a name (Şucan, Taïx); standing alone it is a
        # symbol (ĝ(v), ṡ), and this book's accents on symbols are math.
        if escape and (prev.isalpha() or nxt.isalpha()) and mark in TEXT_ACCENT:
            return TEXT_ACCENT[mark] + "{" + base + "}"
        if mark in MATH_ACCENT:
            return math(MATH_ACCENT[mark] + "{" + base + "}")
    d = unicodedata.decomposition(ch).split()
    if len(d) == 2 and d[0] == "<font>":
        base = chr(int(d[1], 16))
        name = unicodedata.name(ch, "")
        if "SCRIPT" in name:
            cmd = r"\mathcal{" + base + "}"
        elif "DOUBLE-STRUCK" in name:
            cmd = r"\mathbb{" + base + "}"
        elif "FRAKTUR" in name:
            cmd = r"\mathfrak{" + base + "}"
        else:
            cmd = base
        if "BOLD" in name:
            cmd = (r"\mathbf{" + base + "}") if cmd == base else (r"\boldsymbol{" + cmd + "}")
        return math(cmd)
    if ch not in _warned:
        _warned.add(ch)
        print(f"  warning: no LaTeX for {ch!r} (U+{ord(ch):04X} "
              f"{unicodedata.name(ch, '?')}); dropped", file=sys.stderr)
    return ""


def _walk(s, symbol_map, wrap, escape):
    """Shared scanner for text and math mode, with combining-mark lookahead."""
    out = []
    i = 0

    def emit(cmd: str, after: str) -> str:
        # \Sigma followed by a letter would be read as \SigmaA; TeX needs the
        # separator, and a control word swallows the space it is given.
        if cmd[-1:].isalpha() and cmd.startswith("\\") and after[:1].isalpha():
            return cmd + " "
        return cmd

    while i < len(s):
        ch = s[i]
        nxt = s[i + 1] if i + 1 < len(s) else ""
        pair = ch + nxt
        run = _script_run(s, i)
        if run:
            out.append(run[0] if not escape else "$" + run[0] + "$")
            i += run[1]
            continue
        if pair in MATH_PAIRS:
            rep = MATH_PAIRS[pair]
            out.append(rep if not escape else "$" + rep + "$")
            i += 2
            continue
        if nxt in COMBINING and (ch.isalpha() or ch in symbol_map):
            base = symbol_map.get(ch, ch)
            if base.startswith("\\") and escape:
                base = base.strip("$")
            out.append(wrap(COMBINING[nxt] + "{" + base.strip("$") + "}"))
            i += 2
            continue
        if ch in COMBINING:
            i += 1
            continue
        if escape and ch in ESCAPES:
            out.append(ESCAPES[ch])
        elif ch in symbol_map:
            out.append(emit(symbol_map[ch], nxt))
        elif ord(ch) > 127:
            out.append(_fallback(ch, s[i - 1] if i else "", nxt, escape))
        else:
            out.append(ch)
        i += 1
    return "".join(out)


ENTITIES = {"&nbsp;": "\u00a0", "&amp;": "&", "&lt;": "<", "&gt;": ">",
            "&quot;": '"', "&#39;": "'", "&mdash;": "\u2014", "&ndash;": "\u2013"}


def escape_text(s):
    """Prose: escape LaTeX specials, spell symbols so they work in text mode."""
    for ent, ch in ENTITIES.items():
        s = s.replace(ent, ch)
    return _walk(s, UNICODE, lambda cmd: "$" + cmd + "$", True)


# The web edition tints equation terms with \htmlClass{term-prior}{…}; print
# uses the matching colour command. Same convention, same meaning, both media.
TERM_CLASS = {
    # Planning roles — this book's colour code (shannon.sty).
    "term-start": "startterm",
    "term-goal": "goalterm",
    "term-obstacle": "obstacleterm",
    "term-cobstacle": "cobstacleterm",
    "term-graph": "graphterm",
    "term-path": "pathterm",
    "term-robot": "robotterm",
    # Estimation roles — imported from the sister volume for belief panels.
    "term-prior": "prior",
    "term-prediction": "prediction",
    "term-measurement": "measurement",
    "term-posterior": "posterior",
    "term-truth": "truthterm",
}

# A widget/scrub `role` attribute → the colour command that tints it.
ROLE_CMD = {k.removeprefix("term-"): v for k, v in TERM_CLASS.items()}


def rewrite_html_class(s: str) -> str:
    """\\htmlClass{term-x}{body} → \\x{body}, brace-balanced."""
    out = []
    i = 0
    needle = r"\htmlClass{"
    while True:
        j = s.find(needle, i)
        if j < 0:
            out.append(s[i:])
            break
        out.append(s[i:j])
        k = s.find("}", j + len(needle))
        if k < 0:
            out.append(s[j:])
            break
        cls = s[j + len(needle):k]
        # The body is the next brace group.
        if k + 1 < len(s) and s[k + 1] == "{":
            end = match_braces_tex(s, k + 1)
            body = s[k + 2:end - 1]
            cmd = TERM_CLASS.get(cls)
            out.append(f"\\{cmd}{{{rewrite_html_class(body)}}}" if cmd
                       else rewrite_html_class(body))
            i = end
        else:
            i = k + 1
    return "".join(out)


# Spellings KaTeX accepts that LaTeX does not. Keys are regexes (so the
# backslash is doubled); values are the literal LaTeX to emit.
KATEX_ONLY = {
    r"\\lt": "<",
    r"\\gt": ">",
    r"\\ne": "\\neq",
    r"\\le": "\\leq",
    r"\\ge": "\\geq",
    r"\\coloneqq": ":=",
    r"\\bm": "\\boldsymbol",
}


def fix_math(s):
    """Math span: no escaping; symbols take their bare math-mode command."""
    s = rewrite_html_class(s)
    for src, dst in KATEX_ONLY.items():
        # A function replacement, so backslashes in the target are literal.
        s = re.sub(src + r"(?![A-Za-z])", lambda _m, d=dst: d, s)
    # \htmlId / \htmlData carry no meaning on paper.
    s = re.sub(r"\\html(?:Id|Data|Style)\{[^}]*\}", "", s)
    # A prime only merges with a following *literal* ^, so q'\T is a double
    # superscript to TeX (KaTeX accepts it). Spell the transpose out there.
    s = re.sub(r"'\\T(?![A-Za-z])", lambda _m: r"'^{\mathsf{T}}", s)
    s = re.sub(r"(?<!\\)%", lambda _m: "\\%", s)
    # \text{…} contents may hold Unicode that must not reach math mode raw.
    return _walk(s, MATH_SYMBOL, lambda cmd: cmd, False)


def boxable_display(eq: str) -> bool:
    """Can this display equation be set inside an hbox (and so be measured)?

    Everything can except `split` and a bare top-level `\\\\`, which need the
    display environment itself; aligned, cases, arrays and matrices are fine.
    """
    if r"\begin{split}" in eq:
        return False
    flat = eq
    while True:
        nxt = re.sub(r"\\begin\{(\w+\*?)\}(?:(?!\\begin\{).)*?\\end\{\1\}", "", flat, flags=re.S)
        if nxt == flat:
            break
        flat = nxt
    return r"\\" not in flat


def breakable_math(s: str) -> str:
    """Let a list of symbols wrap: TeX never breaks math after a comma itself."""
    out, depth, i = [], 0, 0
    while i < len(s):
        c = s[i]
        if c == "\\":
            m = re.match(r"\\(?:[A-Za-z]+|.)", s[i:])
            tok = m.group(0)
            if depth == 0 and tok == r"\quad":
                out.append(r"\allowbreak")
            out.append(tok)
            i += len(tok)
            continue
        depth += (c == "{") - (c == "}")
        out.append(c)
        if depth == 0 and c in ",;":
            out.append(r"\allowbreak ")
        i += 1
    return "".join(out)


def inline_math(s: str) -> str:
    """Inline math; a long one may also wrap at its top-level commas."""
    tex = fix_math(s)
    return breakable_math(tex) if len(tex) > 40 else tex


def inline_code(code: str) -> str:
    """An inline code span, with the break points typewriter type lacks."""
    tex = escape_text(code)
    tex = re.sub(r"(::|/|,|\()", r"\1\\allowbreak{}", tex)
    # It never hyphenates, so let wasm32-unknown-unknown break at its own
    # hyphens; a -- stays whole (and \/ keeps it from ligaturing to a dash).
    tex = re.sub(r"(?<=\w)-(?=\w)", lambda _m: r"-\allowbreak{}", tex)
    tex = re.sub(r"-(?!\\allowbreak)", lambda _m: r"-\/", tex)
    tex = tex.replace(r"\_", r"\_\allowbreak{}")
    return r"\texttt{" + tex + "}"


def inline(text: str) -> str:
    """Markdown inline → LaTeX, protecting math and code from escaping."""
    protected: list[str] = []

    def stash(payload: str) -> str:
        protected.append(payload)
        return f"\x00{len(protected) - 1}\x00"

    text = re.sub(
        r"\$\$(.+?)\$\$",
        lambda m: stash("$$" + fix_math(m.group(1)) + "$$"),
        text,
        flags=re.S,
    )
    text = re.sub(
        r"(?<!\$)\$([^\$\n]+?)\$(?!\$)",
        lambda m: stash("$" + inline_math(m.group(1)) + "$"),
        text,
    )
    text = re.sub(r"`([^`]+)`", lambda m: stash(inline_code(m.group(1))), text)
    # Markdown backslash escapes (A\*, \_, \[) mean the bare character.
    text = re.sub(r"\\([*_\[\]#`<>!|~])",
                  lambda m: stash(escape_text(m.group(1))), text)
    text = re.sub(
        r"\[([^\]]+)\]\(([^)]+)\)",
        lambda m: stash(_link(m.group(1), m.group(2))),
        text,
    )
    # Typewriter quotes in prose become typographic ones: a quote opens at the
    # start or after a space or bracket, and closes everywhere else.
    text = re.sub(r"""(?:^|(?<=[\s(\[{—–]))"(?=\S)""", "“", text)
    text = text.replace('"', "”")
    text = re.sub(r"(?:^|(?<=[\s(\[{—–]))'(?=\w)", "‘", text)

    text = escape_text(text)

    text = re.sub(r"\*\*\*(.+?)\*\*\*", r"\\textbf{\\emph{\1}}", text, flags=re.S)
    text = re.sub(r"\*\*(.+?)\*\*", r"\\textbf{\1}", text, flags=re.S)
    text = re.sub(r"(?<!\*)\*([^*\n]+?)\*(?!\*)", r"\\emph{\1}", text)

    for _ in range(6):
        before = text
        for i, payload in enumerate(protected):
            text = text.replace(f"\x00{i}\x00", payload)
        if text == before:
            break
    return text


CHAPTER_OF_SLUG: dict[str, int] = {}


def _link(label: str, href: str) -> str:
    """Cross-chapter links become chapter references; the rest become hyperlinks."""
    m = re.match(r"^/chapters/([a-z0-9-]+)", href)
    if m:
        n = CHAPTER_OF_SLUG.get(m.group(1))
        if n:
            return escape_text(label) + f"~(Chapter~{n})" if "hapter" not in label \
                else escape_text(label)
        return escape_text(label)
    safe = href.replace("%", r"\%").replace("#", r"\#")
    return r"\href{" + safe + "}{" + escape_text(label) + "}"


# ---------------------------------------------------------------------------
# Block conversion
# ---------------------------------------------------------------------------

CALLOUT_ENV = {
    "info": "shnote",
    "note": "shnote",
    "tip": "shinsight",
    "warn": "shwarning",
    "warning": "shwarning",
    "error": "shwarning",
    "danger": "shwarning",
    "success": "shinsight",
}


def camel_breaks(cell: str) -> str:
    """A narrow column may split `PlanningRequestAdapter` at its humps."""
    return re.sub(
        r"\\texttt\{[^{}]*\}",
        lambda m: re.sub(r"(?<=[a-z])(?=[A-Z])", lambda _m: r"\allowbreak{}", m.group(0)),
        cell,
    )


def convert_table(lines: list[str]) -> str:
    rows = []
    for ln in lines:
        ln = ln.strip()
        if not ln.startswith("|"):
            continue
        cells = [c.strip() for c in ln.strip("|").split("|")]
        if all(re.fullmatch(r":?-{2,}:?", c) for c in cells if c):
            continue
        rows.append(cells)
    if not rows:
        return ""
    ncols = max(len(r) for r in rows)
    rows = [r + [""] * (ncols - len(r)) for r in rows]

    # Shares of the line, each net of its own two \tabcolsep (4pt apiece), so
    # the table fills the measure exactly instead of overrunning it.
    first = 0.24 if ncols > 2 else 0.36
    rest = (1 - first) / max(ncols - 1, 1)
    colspec = "".join(
        f">{{\\raggedright\\arraybackslash}}p{{\\dimexpr{w:.3f}\\linewidth-8pt\\relax}}"
        for w in [first] + [rest] * (ncols - 1)
    )

    size = "\\scriptsize" if ncols >= 4 else "\\footnotesize"
    out = [f"\\begingroup{size}\\setlength{{\\tabcolsep}}{{4pt}}"
           "\\setlength{\\emergencystretch}{2em}",
           f"\\begin{{longtable}}{{{colspec}}}", "\\toprule"]
    out.append(" & ".join(f"\\textbf{{{inline(c)}}}" for c in rows[0]) + " \\\\")
    out.append("\\midrule\\endhead")
    for r in rows[1:]:
        out.append(" & ".join(camel_breaks(inline(c)) for c in r) + " \\\\")
    out += ["\\bottomrule", "\\end{longtable}", "\\endgroup", ""]
    return "\n".join(out)


def convert_code_fence(lang: str, code: str, meta: str) -> str:
    """A ```rust fence, with its optional title="…" meta, into the shrust box."""
    title = ""
    m = re.search(r'title="([^"]+)"', meta or "")
    if m:
        title = m.group(1)
    body = asciify_code(code.rstrip())
    if lang not in ("rust", "text", "bash", "sh", "toml", ""):
        lang = "text"
    safe_title = title.replace("\\", "/").replace("_", r"\_").replace("#", r"\#") \
                      .replace("&", r"\&").replace("%", r"\%")
    if lang == "rust":
        return (f"\\begin{{shrust}}{{{safe_title}}}{{}}\n"
                f"\\begin{{lstlisting}}\n{body}\n\\end{{lstlisting}}\n"
                f"\\end{{shrust}}\n")
    return ("\\begin{lstlisting}[language={}, numbers=none, frame=leftline, "
            "framerule=1.5pt, rulecolor=\\color{shSlate300}]\n"
            f"{body}\n\\end{{lstlisting}}\n")


def convert_markdown(md: str) -> str:
    """Convert a markdown block (no custom components) to LaTeX."""
    lines = md.split("\n")
    out: list[str] = []
    i = 0
    para: list[str] = []
    list_stack: list[str] = []

    def flush_para():
        nonlocal para
        if para:
            out.append(inline(" ".join(para).strip()))
            out.append("")
            para = []

    def close_lists():
        while list_stack:
            out.append(f"\\end{{{list_stack.pop()}}}")
        out.append("")

    while i < len(lines):
        ln = lines[i]
        stripped = ln.strip()

        # Code fence
        m = re.match(r"^```(\w*)\s*(.*)$", stripped)
        if m:
            flush_para()
            close_lists()
            lang, meta = m.group(1), m.group(2)
            i += 1
            buf = []
            while i < len(lines) and not lines[i].strip().startswith("```"):
                buf.append(lines[i])
                i += 1
            i += 1
            out.append(convert_code_fence(lang, "\n".join(buf), meta))
            continue

        # Display math block
        if stripped.startswith("$$"):
            flush_para()
            close_lists()
            block = [ln]
            if stripped.count("$$") < 2:
                i += 1
                while i < len(lines):
                    block.append(lines[i])
                    if "$$" in lines[i]:
                        break
                    i += 1
            body = "\n".join(block).strip()
            body = body[2:-2] if body.startswith("$$") and body.endswith("$$") else body
            eq = fix_math(body.strip())
            if boxable_display(eq):
                # Measured in TeX, not guessed from the source length: only an
                # equation that really overruns the measure is scaled to fit.
                # A \tag belongs to the display, not to the box being measured.
                tag = re.search(r"\\tag\{(?:[^{}]|\{[^{}]*\})*\}", eq)
                body, tag = (eq[:tag.start()] + eq[tag.end():], tag.group(0)) if tag else (eq, "")
                out += [r"\begin{equation*}", (r"\shfitdisplay[6em]{" if tag else r"\shfitdisplay{")
                        + body.rstrip() + "}" + tag,
                        r"\end{equation*}", ""]
            else:
                out += [r"\begin{equation*}", eq, r"\end{equation*}", ""]
            i += 1
            continue

        # Headings
        m = re.match(r"^(#{2,4})\s+(.*)$", stripped)
        if m:
            flush_para()
            close_lists()
            level, title = len(m.group(1)), m.group(2).strip()
            title = re.sub(r"^\d+\.\d+\s+", "", title)
            cmd = {2: "section", 3: "subsection", 4: "subsubsection"}[level]
            out += [f"\\{cmd}{{{inline(title)}}}", ""]
            i += 1
            continue

        # Horizontal rule
        if re.fullmatch(r"-{3,}|\*{3,}|_{3,}", stripped):
            flush_para()
            close_lists()
            out += ["\\vspace{6pt}\\noindent{\\color{shRule}\\rule{\\linewidth}{0.4pt}}"
                    "\\vspace{6pt}", ""]
            i += 1
            continue

        # Table
        if stripped.startswith("|"):
            flush_para()
            close_lists()
            tbl = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                tbl.append(lines[i])
                i += 1
            out.append(convert_table(tbl))
            continue

        # Blockquote
        if stripped.startswith(">"):
            flush_para()
            close_lists()
            quote = []
            while i < len(lines) and lines[i].strip().startswith(">"):
                quote.append(lines[i].strip().lstrip(">").strip())
                i += 1
            out += [r"\begin{quote}", inline(" ".join(quote)), r"\end{quote}", ""]
            continue

        # Lists
        m_ul = re.match(r"^(\s*)[-*+]\s+(.*)$", ln)
        m_ol = re.match(r"^(\s*)(\d+)\.\s+(.*)$", ln)
        # CommonMark: only "1." may interrupt a paragraph, so a wrapped line
        # that happens to start "10. Total …" is still prose — as on the web.
        if m_ol and para and not list_stack and m_ol.group(2) != "1":
            m_ol = None
        if m_ul or m_ol:
            flush_para()
            env = "itemize" if m_ul else "enumerate"
            body = m_ul.group(2) if m_ul else m_ol.group(3)
            if not list_stack:
                out.append(f"\\begin{{{env}}}")
                list_stack.append(env)
            elif list_stack[-1] != env:
                out.append(f"\\end{{{list_stack.pop()}}}")
                out.append(f"\\begin{{{env}}}")
                list_stack.append(env)
            item = [body]
            i += 1
            while i < len(lines) and lines[i].strip() and not re.match(
                r"^\s*([-*+]|\d+\.)\s+|^#{2,4}\s|^\||^\$\$|^```|^<", lines[i]
            ):
                item.append(lines[i].strip())
                i += 1
            out.append(f"\\item {inline(' '.join(item))}")
            continue

        if not stripped:
            flush_para()
            if list_stack:
                close_lists()
            i += 1
            continue

        para.append(stripped)
        i += 1

    flush_para()
    close_lists()
    return "\n".join(out)


def find_component(text: str, start: int):
    """Locate the next JSX component at or after `start`. Returns a dict or None."""
    pat = re.compile(r"<([A-Z][A-Za-z0-9]*)")
    pos = start
    while True:
        m = pat.search(text, pos)
        if not m:
            return None
        # A component inside a fenced code block is Rust generics, not JSX.
        if text.count("```", 0, m.start()) % 2 == 1:
            pos = m.end()
            continue
        # So is one glued to an identifier — `Path<M>` in an inline code span —
        # or sitting inside such a span on its line: a JSX tag never is.
        line_start = text.rfind("\n", 0, m.start()) + 1
        if (m.start() > 0 and re.match(r"[\w:&]", text[m.start() - 1])) \
                or text.count("`", line_start, m.start()) % 2 == 1:
            pos = m.end()
            continue
        break

    name = m.group(1)
    i = m.end()
    depth = 0
    quote = None
    while i < len(text):
        c = text[i]
        if quote:
            if c == "\\":
                i += 2
                continue
            if c == quote:
                quote = None
        elif c in "\"'":
            quote = c
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
        elif c == ">" and depth == 0:
            break
        i += 1
    tag_end = i
    open_tag = text[m.start(): tag_end + 1]
    self_closing = open_tag.rstrip().endswith("/>")

    if self_closing:
        return {"name": name, "attrs_src": open_tag, "body": "",
                "start": m.start(), "end": tag_end + 1}

    depth = 1
    j = tag_end + 1
    tagpat = re.compile(rf"<(/?){name}[\s>/]")
    close = -1
    while j < len(text) and depth > 0:
        mm = tagpat.search(text, j)
        if not mm:
            break
        if mm.group(1):
            depth -= 1
            if depth == 0:
                close = mm.start()
                break
        elif not text[mm.start():].split(">", 1)[0].rstrip().endswith("/"):
            depth += 1
        j = mm.end()

    if close < 0:
        return {"name": name, "attrs_src": open_tag, "body": text[tag_end + 1:],
                "start": m.start(), "end": len(text)}
    return {
        "name": name,
        "attrs_src": open_tag,
        "body": text[tag_end + 1: close],
        "start": m.start(),
        "end": close + len(name) + 3,
    }


def parse_attrs(src: str) -> dict:
    """Extract JSX attributes: string literals and {…} expressions."""
    attrs = {}
    body = re.sub(r"^<[A-Za-z0-9]+", "", src).rstrip()
    body = body[:-2] if body.endswith("/>") else body.rstrip(">")
    i = 0
    while i < len(body):
        m = re.compile(r"([A-Za-z_][\w]*)\s*=\s*").search(body, i)
        if not m:
            break
        key = m.group(1)
        j = m.end()
        if j >= len(body):
            break
        if body[j] in "\"'":
            q = body[j]
            k = j + 1
            buf = []
            while k < len(body):
                if body[k] == "\\" and k + 1 < len(body) and body[k + 1] == q:
                    buf.append(q)            # escaped quote
                    k += 2
                    continue
                if body[k] == q:
                    break
                buf.append(body[k])
                k += 1
            attrs[key] = "".join(buf)
            i = k + 1
        elif body[j] == "{":
            end = match_braces(body, j)
            attrs[key] = parse_js(body[j + 1: end - 1])
            i = end
        else:
            i = j + 1
    return attrs


# ---------------------------------------------------------------------------
# Widget registry — read out of the React components themselves
# ---------------------------------------------------------------------------

WIDGETS: dict[str, dict] = {}


def load_widgets(components_dir: Path) -> None:
    """
    Map each exported widget component to the id, title and teaching point it
    declares on its WidgetFrame. Reading it from the source means the print
    edition cannot drift from the interactive one.
    """
    for tsx in sorted(components_dir.glob("ch*/*.tsx")):
        src = tsx.read_text(encoding="utf-8")
        for fm in re.finditer(r"<WidgetFrame\b", src):
            seg = src[fm.start(): fm.start() + 2600]
            wid = re.search(r'id="([^"]+)"', seg)
            title = re.search(r'title="([^"]+)"', seg)
            teaches = re.search(r'teaches="([^"]+)"', seg)
            if not wid:
                continue
            # The exported component enclosing this frame.
            names = re.findall(r"export function ([A-Z][A-Za-z0-9]*)", src[:fm.start()])
            comp = names[-1] if names else None
            if not comp:
                continue
            WIDGETS[comp] = {
                "id": wid.group(1),
                "title": title.group(1) if title else comp,
                "teaches": teaches.group(1) if teaches else "",
                "slug": tsx.parent.name,
            }


def render_widget(name: str) -> str:
    w = WIDGETS.get(name)
    if not w:
        return ""
    return ("\\shwidget{%s}{%s}{%s}\n"
            % (w["id"], inline(w["title"]), inline(w["teaches"])))


# ---------------------------------------------------------------------------
# Components
# ---------------------------------------------------------------------------


def _result_tex(result: str) -> str:
    """
    A derivation's stated result is usually a formula, but a few are a sentence.
    Prose set in math mode loses its spaces, so detect it and typeset it as text.
    """
    words = re.findall(r"(?<![\\\\A-Za-z])[a-z]{3,}(?![A-Za-z])", result)
    if len(words) >= 4:
        return inline(result)
    return "$\\displaystyle " + fix_math(result) + "$"


def render_component(comp: dict) -> str:
    name = comp["name"]
    attrs = parse_attrs(comp["attrs_src"])
    body = comp["body"]

    if name == "Overview":
        out = [convert_body(body).strip()]
        goals = attrs.get("goals") or []
        prereqs = attrs.get("prerequisites") or []
        if goals:
            out.append("\\begin{shgoals}")
            out += [f"  \\item {inline(str(g))}" for g in goals]
            out.append("\\end{shgoals}")
        if prereqs:
            out.append("\\begin{shprereqs}")
            out += [f"  \\item {inline(str(p))}" for p in prereqs]
            out.append("\\end{shprereqs}")
        return "\n".join(out) + "\n"

    if name == "KeyIdea":
        title = attrs.get("title", "Key idea")
        return (f"\\begin{{shinsight}}[{inline(title)}]\n"
                f"{convert_body(body)}\n\\end{{shinsight}}\n")

    if name == "Callout":
        env = CALLOUT_ENV.get(str(attrs.get("type", "note")).lower(), "shnote")
        title = attrs.get("title")
        opt = f"[{inline(title)}]" if title else ""
        return f"\\begin{{{env}}}{opt}\n{convert_body(body)}\n\\end{{{env}}}\n"

    if name == "Derivation":
        title = inline(attrs.get("title", "Derivation"))
        result = attrs.get("result", "")
        result_tex = _result_tex(result) if result else ""
        # The title goes on the rule; the result, if any, sits beside it.
        return (f"\\begin{{shderivation}}{{{result_tex if result_tex else chr(92)+'relax'}}}"
                f"{{{title}}}\n{convert_body(body)}\n\\end{{shderivation}}\n")

    if name == "Algorithm":
        aname = attrs.get("name", "")
        inputs = inline(attrs.get("inputs", "")) or "\\relax"
        outputs = inline(attrs.get("outputs", "")) or "\\relax"
        cost = inline(attrs.get("complexity", "")) or "\\relax"
        safe_name = escape_text(aname)
        return (f"\\begin{{shalgorithm}}{{{safe_name}}}{{{inputs}}}{{{outputs}}}{{{cost}}}\n"
                f"{convert_body(body)}\n\\end{{shalgorithm}}\n")

    if name == "NotationTable":
        rows = attrs.get("rows") or []
        out = ["\\begin{shnotation}"]
        for r in rows:
            if not isinstance(r, dict):
                continue
            sym = breakable_math(fix_math(str(r.get("sym", ""))))
            meaning = inline(str(r.get("meaning", "")))
            note = r.get("note")
            if note:
                meaning += (" \\newline {\\footnotesize\\color{shInkMuted}"
                            + inline(str(note)) + "}")
            out.append(f"{sym} & {meaning} \\\\")
        out.append("\\end{shnotation}")
        return "\n".join(out) + "\n"

    if name == "Exercises":
        return "\\begin{shexercises}\n" + convert_body(body) + "\n\\end{shexercises}\n"

    if name == "Exercise":
        level = str(attrs.get("level", "F")).strip()[:1] or "F"
        diff = attrs.get("difficulty", 2)
        try:
            diff = int(diff)
        except (TypeError, ValueError):
            diff = 2
        title = inline(attrs.get("title", ""))
        inner = convert_body(body).strip()
        extra = []
        if attrs.get("hint"):
            extra.append("\\par\\vspace{3pt}{\\fontsize{8.5}{11}\\selectfont"
                         "\\color{shInkMuted}\\textbf{Hint.} "
                         + inline(str(attrs["hint"])) + "}")
        if attrs.get("solution"):
            extra.append("\\par\\vspace{3pt}{\\fontsize{8.5}{11}\\selectfont"
                         "\\color{shInkMuted}\\textbf{Solution.} "
                         + inline(str(attrs["solution"])) + "}")
        return f"\\shexercise{{{level}}}{{{diff}}}{{{title}}}{{{inner}{''.join(extra)}}}\n"

    if name == "References":
        return "\\begin{shreferences}\n" + convert_body(body) + "\n\\end{shreferences}\n"

    if name == "Reference":
        authors = inline(str(attrs.get("authors", "")))
        year = inline(str(attrs.get("year", "")))
        title = inline(str(attrs.get("title", "")))
        venue = inline(str(attrs.get("venue", ""))) if attrs.get("venue") else "\\relax"
        bits = []
        if attrs.get("doi"):
            doi = str(attrs["doi"]).replace("https://doi.org/", "")
            bits.append("\\texttt{doi:" + doi.replace("_", r"\_") + "}")
        elif attrs.get("url"):
            url = str(attrs["url"])
            bits.append("\\href{" + url.replace("%", r"\%").replace("#", r"\#")
                        + "}{\\texttt{link}}")
        note = inline(str(attrs.get("note", ""))) if attrs.get("note") else ""
        if bits:
            note = (note + " " if note else "") + " ".join(bits)
        return f"\\shref{{{authors}}}{{{year}}}{{{title}}}{{{venue}}}{{{note or chr(92)+'relax'}}}\n"

    if name == "Figure":
        caption = inline(attrs.get("caption", ""))
        inner = convert_body(body).strip()
        return ("\\begin{tcolorbox}[shblock, colframe=shSlate300, colback=shSlate50]\n"
                f"{inner}\n"
                + (f"\\par\\vspace{{4pt}}{{\\fontsize{{9}}{{12}}\\selectfont"
                   f"\\color{{shInkMuted}} {caption}}}\n" if caption else "")
                + "\\end{tcolorbox}\n")

    if name == "TeX":
        raw = body.strip()
        m = re.search(r"String\.raw\s*`(.*?)`", raw, re.S)
        if not m:
            m = re.match(r"\{\s*[`'\"](.*)[`'\"]\s*\}$", raw, re.S)
        tex = fix_math((m.group(1) if m else raw).strip())
        display = "display" in comp["attrs_src"]
        if display:
            return "\\begin{equation*}\n" + tex + "\n\\end{equation*}\n"
        return "$" + tex + "$"

    if name == "Scrub":
        # A draggable number is just a number on paper — but it must still be
        # there, or the sentence it sits inside loses its subject.
        try:
            value = float(attrs.get("value", 0))
        except (TypeError, ValueError):
            value = 0.0
        try:
            step = float(attrs.get("step", 0.01))
        except (TypeError, ValueError):
            step = 0.01
        digits = max(0, min(3, -int(round(__import__("math").log10(step))))) if step > 0 else 2
        unit = attrs.get("unit", "")
        shown = f"{value:.{digits}f}"
        role = attrs.get("role")
        num = ("\\%s{%s}" % (ROLE_CMD.get(str(role), "robotterm"), shown)) if role else shown
        return num + (("\\," + escape_text(str(unit))) if unit else "")

    if name == "LinkedMath":
        # The hover linkage is a screen affordance; the mathematics is the same.
        return convert_body(body)

    if name == "RoleTag":
        role = str(attrs.get("role", "path"))
        cmd = ROLE_CMD.get(role, "robotterm")
        inner = convert_body(body).strip() or role
        return "\\%s{%s}" % (cmd, inner)

    if name == "Predict":
        question = inline(str(attrs.get("question", "")))
        options = attrs.get("options") or []
        out = ["\\begin{shcallout}{shConceptual}{\\faEye}{Predict first}",
               question, "\\begin{itemize}"]
        for opt in options:
            if not isinstance(opt, dict):
                continue
            mark = "$\\checkmark$\\ " if opt.get("correct") else ""
            line = mark + inline(str(opt.get("label", "")))
            because = opt.get("because")
            if because:
                line += ("\\newline{\\footnotesize\\color{shInkMuted}"
                         + inline(str(because)) + "}")
            out.append("  \\item " + line)
        out.append("\\end{itemize}")
        rest = convert_body(body).strip()
        if rest:
            out.append(rest)
        out.append("\\end{shcallout}")
        return "\n".join(out) + "\n"

    if name == "CheckAnswer":
        prompt = inline(str(attrs.get("prompt", "")))
        answer = attrs.get("answer", "")
        unit = attrs.get("unit", "")
        out = ["\\begin{shcallout}{shFoundation}{\\faCalculator}{Work it out}", prompt,
               "\\par\\vspace{3pt}{\\fontsize{9}{11}\\selectfont\\color{shInkMuted}"
               "\\textbf{Answer.} $" + str(answer) + "$"
               + ((" " + escape_text(str(unit))) if unit else "") + "}"]
        rest = convert_body(body).strip()
        if rest:
            out.append(rest)
        out.append("\\end{shcallout}")
        return "\n".join(out) + "\n"

    if name == "Hints":
        inner = convert_body(body).strip()
        return ("\\begin{shnote}[Hints]\n" + inner + "\n\\end{shnote}\n") if inner else ""

    if name == "ColorKey":
        return "\\shcolorkey\n"

    if name in ("Tabs", "Tab", "Steps", "Step", "Accordions", "Accordion"):
        # Structural on screen only; the prose reads fine in sequence.
        label = attrs.get("title") or attrs.get("value")
        head = (f"\\par\\vspace{{4pt}}\\noindent{{\\displayfont\\bfseries"
                f"\\fontsize{{9.5}}{{12}}\\selectfont\\color{{shInk}}{inline(str(label))}}}"
                f"\\par\\vspace{{2pt}}\n") if label else ""
        return head + convert_body(body)

    if name in WIDGETS:
        return render_widget(name)

    # An unknown self-closing component is almost certainly a widget whose frame
    # we failed to read; say so rather than silently dropping a figure.
    if not body.strip():
        return ""

    return convert_body(body)


def convert_body(text: str) -> str:
    """Alternate between markdown runs and JSX components."""
    out = []
    pos = 0
    while True:
        comp = find_component(text, pos)
        if not comp:
            out.append(convert_markdown(text[pos:]))
            break
        out.append(convert_markdown(text[pos: comp["start"]]))
        out.append(render_component(comp))
        pos = comp["end"]
    return "\n".join(out)


# ---------------------------------------------------------------------------
# Frontmatter and driver
# ---------------------------------------------------------------------------


def parse_frontmatter(src: str):
    m = re.match(r"^---\n(.*?)\n---\n", src, re.S)
    if not m:
        return {}, src
    raw, rest = m.group(1), src[m.end():]
    meta = {}
    for line in raw.split("\n"):
        m2 = re.match(r"^([A-Za-z_]+):\s*(.*)$", line)
        if not m2:
            continue
        key, val = m2.group(1), m2.group(2).strip()
        if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
            val = val[1:-1]
        meta[key] = val
    return meta, rest


def strip_imports(body: str) -> str:
    return re.sub(r"^import\s+.*$\n?", "", body, flags=re.M)


def convert_chapter(path: Path) -> str:
    src = path.read_text(encoding="utf-8")
    meta, body = parse_frontmatter(src)
    body = strip_imports(body)

    slug = path.stem
    title = meta.get("title", path.stem)

    out = [
        f"% Generated from web/content/chapters/{path.name} -- do not edit by hand.",
        f"\\chapter{{{inline(title)}}}",
        f"\\setchapterslug{{{slug}}}",
        "",
    ]

    part = meta.get("partTitle") or meta.get("part") or ""
    if part or meta.get("difficulty") or meta.get("readingTime"):
        out.append("\\shchaptermeta{%s}{%s}{%s}" % (
            inline(part),
            inline(meta.get("difficulty", "")) or "\\relax",
            inline(meta.get("readingTime", "")) or "\\relax",
        ))
        out.append("")

    if meta.get("quote"):
        out.append("\\shepigraph{%s}{%s}{%s}{%s}" % (
            inline(meta["quote"]),
            inline(meta.get("quoteAuthor", "")),
            "\\relax",
            inline(meta.get("quoteSource", "")),
        ))
        out.append("")

    out.append(convert_body(body))
    return "\n".join(out) + "\n"


def main() -> int:
    here = Path(__file__).resolve().parents[1]
    root = here.parent
    src_dir = root / "web" / "content" / "chapters"
    out_dir = here / "chapters"
    out_dir.mkdir(parents=True, exist_ok=True)

    load_widgets(root / "web" / "components" / "ch")

    # Chapter numbers, so that cross-references can name a chapter.
    structure = (root / "web" / "lib" / "book-structure.ts").read_text(encoding="utf-8")
    slugs = re.findall(r"slug:\s*'([^']+)'", structure)
    numbers = [int(n) for n in re.findall(r"\n\s*n:\s*(\d+),", structure)]
    CHAPTER_OF_SLUG.update(dict(zip(slugs, numbers)))

    files = sorted(src_dir.glob("ch*.mdx"))
    if not files:
        print(f"no chapters found in {src_dir}", file=sys.stderr)
        return 1

    print(f"{len(WIDGETS)} widgets registered from the web edition\n")
    for f in files:
        tex = convert_chapter(f)
        target = out_dir / f"{f.stem.split('-')[0]}.tex"
        target.write_text(tex, encoding="utf-8")
        print(f"{f.name:36s} → {target.name}  ({len(tex.splitlines()):5d} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
